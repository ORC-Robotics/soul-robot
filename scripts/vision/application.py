"""Lifecycle da aplicação da câmera inferior."""

import os
import signal
import time
import cv2  # type: ignore
import numpy as np

from . import stream_display
try:
    from silver_dataset_recorder import SilverDatasetRecorder
except ImportError:
    SilverDatasetRecorder = None
from .calibration_capture import CalibrationCapture
from .gap_validation import GapValidator, read_json_snapshot, FORWARD_STATUS_PATH
from .line_presence import draw_near_presence_overlay
from .line_control import LineFollowerController
from .maneuver_state import LineManeuverState
from .status_publisher import LineStatusPublisher
from .camera import (
    create_camera,
    rectangle_values,
    tune_camera_image,
)
from .camera_config import (
    DISPLAY_MODE_LINE,
    GPIO,
    GREEN_CAPTURE_REQUEST_PATH,
    GREEN_PROCESSING_ENABLED,
    LEGACY_LINE_DEBUG_ENABLED,
    LIGHT_PIN_BOARD,
    LIMIAR_CURVA_VERDE_INICIADA,
    Picamera2,
    QUADROS_CENTRALIZADO_PARA_CONCLUIR,
    SILVER_DETECTION_ENABLED,
    SNAPSHOT_FRAME_FPS,
    STATUS_FPS,
    Transform,
    VISIBLE_GREEN_INTERPRETATIONS,
    camera_role_publishes_line_status,
    configured_camera_indices,
    log_camera_inventory,
    require_camera_assignment,
    resolve_camera_assignments,
)
from .fusion_guidance import (
    apply_green_candidate_fusion_hold,
    capture_valid_fusion_command,
    draw_fusion_style_line_overlay,
    extract_line_diagnostics,
    fusion_style_blind_search_direction,
    green_maneuver_is_geometrically_complete,
    select_confirmed_green_direction,
    update_fusion_style_history,
    update_green_control_telemetry,
    update_green_fusion_target_hold,
    update_green_rearm_state,
)
from .geometric_guidance import (
    extract_gap_geometric_guidance,
)
from .green_detection import (
    GreenObservationTracker,
    analyze_green_marker_contours,
    build_green_status,
    create_green_mask_stages,
    find_green_candidates,
    save_green_capture,
)
from .line_control import (
    draw_line_control_overlay,
    update_green_maneuver_state,
    virtual_far_line_is_visible,
    virtual_raw_line_is_visible,
)
from .line_masks import (
    create_filtered_line_mask,
    create_line_candidate_mask,
    create_structural_line_mask,
    resolve_vision_geometry,
)
from .illumination_correction import line_illumination_data
from .normal_trajectory import (
    draw_normal_trajectory_overlay,
)
from .silver_detection import (
    SilverShadowMonitor,
    empty_silver_shadow_status,
)
from .stream_display import (
    create_display_frame,
    draw_green_candidate_overlays,
    draw_green_rejection_details,
    draw_green_roi_overlays,
    draw_line_illumination_overlay,
    draw_line_mode_green_overlays,
    draw_silver_shadow_overlay,
    encode_frame,
    get_display_mode,
    handle_signal,
    parse_camera_profile,
    publish_stream_frame,
    save_frame,
    start_stream_server,
    stream_frame_is_due,
)
from .virtual_sensors import (
    draw_virtual_sensor_geometry,
    read_virtual_line_sensors,
    resolve_virtual_sensor_geometry,
    virtual_sensor_is_active,
    virtual_sensor_trust_is_active,
)


class DownwardCameraApplication:
    """Gerencia inicialização, loop e encerramento da câmera inferior."""

    def run(self):
        signal.signal(signal.SIGINT, handle_signal)
        signal.signal(signal.SIGTERM, handle_signal)
        camera_profile = parse_camera_profile()
        vision_profile = camera_profile["vision"]
        camera_details = {}
        status_publisher = LineStatusPublisher()
        self.dataset_recorder = None
        silver_shadow_monitor = None
        silver_shadow_status = empty_silver_shadow_status(
            "Detector da faixa prata desativado por configuração."
        )

        if GPIO is None or Picamera2 is None or Transform is None:
            error_message = "Dependências GPIO, libcamera ou Picamera2 não encontradas."
            print(error_message, flush=True)
            status_publisher.save_status(
                0.0,
                camera_profile,
                camera_details,
                active=False,
                error_message=error_message,
            )
            return 1

        status_publisher.save_status(0.0, camera_profile, camera_details)
        stream_server = None
        picam2 = None
        camera_started = False
        light_ready = False
        calibration_capture = None

        try:
            camera_indices = configured_camera_indices()
            camera_infos = Picamera2.global_camera_info()
            camera_assignments = resolve_camera_assignments(
                camera_infos,
                camera_indices,
            )
            log_camera_inventory(camera_infos, camera_assignments)
            selected_camera = require_camera_assignment(
                camera_assignments,
                camera_profile["role"],
            )
            camera_details["cameraIndex"] = selected_camera["index"]
            line_ipc_enabled = camera_role_publishes_line_status(
                camera_profile["role"]
            )

            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(LIGHT_PIN_BOARD, GPIO.OUT)
            GPIO.output(LIGHT_PIN_BOARD, GPIO.HIGH)
            light_ready = True

            stream_server = start_stream_server()
            picam2, camera_format, camera_details = create_camera(
                camera_profile,
                selected_camera["index"],
            )
            picam2.start()
            tune_camera_image(picam2)
            capture_metadata = picam2.capture_metadata()
            camera_details["scalerCrop"] = rectangle_values(
                capture_metadata["ScalerCrop"]
            )
            print(
                f"Câmera id={camera_details['cameraId']} "
                f"modelo={camera_details['cameraModel']} "
                f"transformação={camera_details['transform']} "
                f"ScalerCrop={camera_details['scalerCrop']}.",
                flush=True,
            )
            camera_started = True
            calibration_capture = CalibrationCapture(picam2, camera_profile, camera_details)

            previous_time = time.monotonic()
            last_stream_time = 0.0
            last_snapshot_time = 0.0
            last_status_time = 0.0
            smoothed_fps = 0.0
            line_sequence = 0
            green_tracker = GreenObservationTracker()
            line_controller = LineFollowerController()
            gap_validator = GapValidator()
            maneuver_state = LineManeuverState()
            maneuver_state.fusion_target_history = None

            # Estado persistente das manobras sinalizadas por verde.
            maneuver_state.green_direction = "NENHUMA"
            maneuver_state.green_curve_started = False
            maneuver_state.green_centered_frames = 0
            maneuver_state.green_active_frames = 0

             # Estado persistente da travessia de gap.
            maneuver_state.gap_forward_active = False
            maneuver_state.gap_fusion_reacquire_active = False
            maneuver_state.gap_forward_frames = 0
            maneuver_state.gap_reacquire_frames = 0
            maneuver_state.gap_line_lost_seen = False
            maneuver_state.gap_recent_near_frames = 0

            # Impede que o mesmo marcador verde seja aceito novamente.
            maneuver_state.green_armed = True
            maneuver_state.green_clear_frames = 0

            # O hold conserva somente o último comando Fusion realmente publicado.
            # Após dois frames, o mesmo candidato precisa desaparecer antes de abrir
            # uma nova janela.
            maneuver_state.last_applied_fusion_command = None
            maneuver_state.green_candidate_hold_frames = 0
            maneuver_state.green_candidate_hold_blocked = False

            # Conserva apenas o último target do GREEN durante uma interrupção de
            # até dois frames; uma perda maior libera a recuperação direcional.
            maneuver_state.last_green_fusion_line = None
            maneuver_state.green_fusion_target_missing_frames = 0

            green_processing_enabled = bool(
                vision_profile.get("green_detection_enabled", False)
                and GREEN_PROCESSING_ENABLED
            )
            print(
                "Visão verde para overlay e telemetria: "
                f"{'ligada' if green_processing_enabled else 'desligada'}.",
                flush=True,
            )

            # A coleta compartilha os frames da câmera e falha sem interromper a visão.
            if SilverDatasetRecorder is not None:
                try:
                    self.dataset_recorder = SilverDatasetRecorder("down")
                except Exception as error:
                    print(f"Coleta do dataset inferior indisponível: {error}", flush=True)

            if SILVER_DETECTION_ENABLED:
                silver_shadow_monitor = SilverShadowMonitor.from_camera_model("down")
            else:
                print(
                    "Detector da faixa prata desativado por configuração; "
                    "nenhum marcador cinza será publicado para a missão.",
                    flush=True,
                )

            while stream_display.running:
                calibration_requested = calibration_capture.before_frame()
                green_capture_requested = os.path.isfile(GREEN_CAPTURE_REQUEST_PATH)
                green_capture_metadata = {}
                if green_capture_requested or calibration_requested:
                    # A captura preserva metadados do mesmo frame usado no diagnóstico.
                    camera_request = picam2.capture_request()
                    try:
                        raw_frame = camera_request.make_array("main")
                        green_capture_metadata = camera_request.get_metadata()
                    finally:
                        camera_request.release()
                else:
                    raw_frame = picam2.capture_array()

                if self.dataset_recorder is not None:
                    try:
                        self.dataset_recorder.submit(raw_frame)
                    except Exception as error:
                        print(f"Coleta do dataset inferior desativada após erro inesperado: {error}", flush=True)
                        self.dataset_recorder = None

                frame_height = raw_frame.shape[0]
                vision_geometry = resolve_vision_geometry(
                    frame_height,
                    vision_profile,
                )
                # A zona acima do FAR mostra partes do chassi e não pertence à
                # pista. Usar a geometria do sensor evita outro limite Y.
                dead_zone_end_y = resolve_virtual_sensor_geometry(
                    raw_frame.shape
                )["far"]["left"]["y0"]
                line_vision_started = time.perf_counter()
                line_timings = {}

                filtered_mask, roi_start_y, specular_repair_status = (
                    create_filtered_line_mask(
                        raw_frame,
                        vision_profile,
                        camera_format,
                        return_repair_status=True,
                        timings=line_timings,
                    )
                )
                structural_mask = create_structural_line_mask(
                    filtered_mask,
                    roi_start_y,
                    vision_geometry["structural_end_y"],
                    dead_zone_end_y,
                )
                contours_started = time.perf_counter()
                line_candidate_mask, accepted_line_contours = (
                    create_line_candidate_mask(
                        structural_mask,
                        vision_profile,
                        return_accepted_contours=True,
                    )
                )
                line_timings["contoursMs"] = (
                    time.perf_counter() - contours_started
                ) * 1000.0
                line_vision_before_green_ms = (
                    time.perf_counter() - line_vision_started
                ) * 1000.0

                green_candidates = []
                green_rejected = []
                green_mask = np.zeros(
                    (vision_geometry["green_end_y"], raw_frame.shape[1]),
                    dtype=np.uint8,
                )
                green_interpretation = analyze_green_marker_contours(
                    [],
                    structural_mask,
                )
                green_overlay_roi_interpretation = green_interpretation
                green_processing_started = time.perf_counter()
                if green_processing_enabled:
                    green_mask, green_candidates, green_rejected = (
                        find_green_candidates(
                            raw_frame,
                            vision_geometry["green_end_y"],
                            camera_format,
                            green_start_y=dead_zone_end_y,
                        )
                    )
                    if vision_profile.get("line_exclude_green", False):
                        # O detector e sua associação conservam o preto original.
                        # Somente a máscara entregue ao seguidor exclui o verde.
                        line_candidate_mask, accepted_line_contours = create_line_candidate_mask(
                            structural_mask, vision_profile,
                            return_accepted_contours=True, green_mask=green_mask,
                        )
                    # A classificação usa o preto estrutural local, não uma
                    # referência de direção ou posição destinada ao controle.
                    green_association_mask = structural_mask.copy()
                    green_association_mask[:dead_zone_end_y, :] = 0
                    green_association_mask[
                        vision_geometry["green_end_y"]:,
                        :,
                    ] = 0
                    useful_height = min(
                        green_association_mask.shape[0],
                        green_mask.shape[0],
                    )
                    useful_width = min(
                        green_association_mask.shape[1],
                        green_mask.shape[1],
                    )
                    association_region = green_association_mask[
                        :useful_height,
                        :useful_width,
                    ]
                    association_region[
                        green_mask[:useful_height, :useful_width] > 0
                    ] = 0
                    green_interpretation = analyze_green_marker_contours(
                        [candidate["contour"] for candidate in green_candidates],
                        green_association_mask,
                    )
                    green_overlay_roi_interpretation = analyze_green_marker_contours(
                        [
                            candidate["contour"]
                            for candidate in green_candidates + green_rejected
                        ],
                        green_association_mask,
                    )
                green_processing_ms = (
                    time.perf_counter() - green_processing_started
                ) * 1000.0

                line_timestamp = time.time()
                line_sequence += 1
                if (
                    SILVER_DETECTION_ENABLED
                    and silver_shadow_monitor is not None
                ):
                    silver_shadow_status = silver_shadow_monitor.process(
                        raw_frame,
                        line_sequence,
                        line_timestamp,
                    )
                green_raw_interpretation = green_interpretation["interpretation"]
                green_tracker_result = green_tracker.update(
                    line_sequence,
                    green_raw_interpretation,
                    time.perf_counter(),
                )
                green_status = build_green_status(
                    green_candidates,
                    len(green_rejected),
                    green_interpretation,
                    green_tracker_result,
                    green_processing_ms,
                )
                green_status["greenRawInterpretation"] = green_raw_interpretation
                green_status["greenPathBlackValid"] = bool(
                    green_interpretation["path_black_valid"]
                )

                # Um verde confirmado é aceito apenas quando o sistema está armado
                # e nenhuma outra direção verde está sendo executada.
                confirmed_green_direction = select_confirmed_green_direction(
                    maneuver_state.green_armed,
                    maneuver_state.green_direction,
                    green_status,
                )
                green_accepted_this_frame = confirmed_green_direction is not None
                if confirmed_green_direction is not None:
                    if confirmed_green_direction == "ESQUERDA":
                        maneuver_state.green_direction = "ESQUERDA"
                        maneuver_state.green_curve_started = False
                        maneuver_state.green_centered_frames = 0
                        maneuver_state.green_active_frames = 0
                        line_controller.line_search_tracker.stop()
                        maneuver_state.green_armed = False
                        maneuver_state.green_clear_frames = 0

                    elif confirmed_green_direction == "DIREITA":
                        maneuver_state.green_direction = "DIREITA"
                        maneuver_state.green_curve_started = False
                        maneuver_state.green_centered_frames = 0
                        maneuver_state.green_active_frames = 0
                        line_controller.line_search_tracker.stop()
                        maneuver_state.green_armed = False
                        maneuver_state.green_clear_frames = 0

                virtual_sensors = read_virtual_line_sensors(
                    line_candidate_mask,
                    maneuver_state.green_direction,
                    maneuver_state.green_curve_started,
                )
                near_center_visible = virtual_sensor_is_active(
                    virtual_sensors["nearCenter"]
                )
                raw_line_visible = virtual_raw_line_is_visible(
                    virtual_sensors
                )
                green_timeout_state = update_green_maneuver_state(
                    maneuver_state.green_direction,
                    maneuver_state.green_active_frames,
                    raw_line_visible,
                )
                sensor_recovery_requested = False
                if green_timeout_state["timedOut"]:
                    maneuver_state.green_direction = green_timeout_state["direction"]
                    maneuver_state.green_active_frames = green_timeout_state["activeFrames"]
                    maneuver_state.green_curve_started = False
                    maneuver_state.green_centered_frames = 0
                    search_direction = green_timeout_state["searchDirection"]
                    if search_direction is not None:
                        line_controller.line_search_tracker.start(search_direction)
                    else:
                        sensor_recovery_requested = True
                    # Remove a máscara verde já no mesmo frame do timeout.
                    virtual_sensors = read_virtual_line_sensors(
                        line_candidate_mask,
                        maneuver_state.green_direction,
                        maneuver_state.green_curve_started,
                    )
                else:
                    maneuver_state.green_direction = green_timeout_state["direction"]
                    maneuver_state.green_active_frames = green_timeout_state["activeFrames"]

                fusion_preferred_direction = {
                    "ESQUERDA": "LEFT",
                    "DIREITA": "RIGHT",
                }.get(maneuver_state.green_direction)
                fusion_history_for_frame = maneuver_state.fusion_target_history
                if (
                    green_accepted_this_frame
                    and isinstance(maneuver_state.fusion_target_history, dict)
                ):
                    # Um novo GREEN representa uma troca intencional de ramo.
                    # Somente o guard antigo é neutralizado; as demais métricas
                    # do histórico Fusion continuam disponíveis neste frame.
                    fusion_history_for_frame = dict(maneuver_state.fusion_target_history)
                    fusion_history_for_frame["pivotDirectionGuard"] = "NONE"
                    fusion_history_for_frame["pivotDirectionGuardActive"] = False
                    fusion_history_for_frame[
                        "pivotDirectionGuardRejectedOpposite"
                    ] = False
                line_vision_after_green_started = time.perf_counter()
                normal_trajectory, fusion_style_line = extract_line_diagnostics(
                    line_candidate_mask,
                    camera_profile["role"],
                    LEGACY_LINE_DEBUG_ENABLED,
                    previous_fusion_line=fusion_history_for_frame,
                    accepted_contours=accepted_line_contours,
                    preferred_direction=fusion_preferred_direction,
                )
                maneuver_state.fusion_target_history = update_fusion_style_history(
                    fusion_history_for_frame,
                    fusion_style_line,
                )
                fusion_blind_search_direction = (
                    fusion_style_blind_search_direction(
                        maneuver_state.fusion_target_history
                    )
                )
                line_timings["normalTrajectoryMs"] = normal_trajectory[
                    "processingMs"
                ]
                line_timings["fusionStyleMs"] = fusion_style_line[
                    "processingMs"
                ]
                green_fusion_target_state = update_green_fusion_target_hold(
                    maneuver_state.green_direction,
                    fusion_style_line,
                    maneuver_state.last_green_fusion_line,
                    maneuver_state.green_fusion_target_missing_frames,
                )
                fusion_style_line = green_fusion_target_state["fusionLine"]
                maneuver_state.last_green_fusion_line = green_fusion_target_state[
                    "previousValidFusionLine"
                ]
                maneuver_state.green_fusion_target_missing_frames = green_fusion_target_state[
                    "missingFrames"
                ]
                green_fusion_recovery_direction = green_fusion_target_state[
                    "recoveryDirection"
                ]
                if green_fusion_recovery_direction is not None:
                    # O target preferido não voltou na janela curta. A manobra
                    # GREEN termina e a busca existente começa no mesmo lado.
                    maneuver_state.green_direction = "NENHUMA"
                    maneuver_state.green_curve_started = False
                    maneuver_state.green_centered_frames = 0
                    maneuver_state.green_active_frames = 0
                    line_controller.line_search_tracker.start(
                        green_fusion_recovery_direction
                    )
                    virtual_sensors = read_virtual_line_sensors(
                        line_candidate_mask,
                        maneuver_state.green_direction,
                        maneuver_state.green_curve_started,
                    )
                    near_center_visible = virtual_sensor_is_active(
                        virtual_sensors["nearCenter"]
                    )
                    raw_line_visible = virtual_raw_line_is_visible(
                        virtual_sensors
                    )
                line_vision_ms = line_vision_before_green_ms + (
                    time.perf_counter() - line_vision_after_green_started
                ) * 1000.0

                geometric_guidance = extract_gap_geometric_guidance(
                    line_candidate_mask,
                    maneuver_state.gap_forward_active,
                    maneuver_state.green_direction,
                    near_center_visible,
                    forward_control_trusted=(
                        virtual_sensor_trust_is_active(
                            virtual_sensors,
                            "farTrusted",
                        )
                        or virtual_sensor_trust_is_active(
                            virtual_sensors,
                            "mediumTrusted",
                        )
                    ),
                )
                line_control_started = time.perf_counter()
                bottom_far_present = virtual_far_line_is_visible(
                    virtual_sensors
                )
                gap_blind_search_requested = gap_validator.process_frame(
                    line_candidate_mask, maneuver_state, line_controller, line_sequence,
                    line_timestamp, read_json_snapshot(FORWARD_STATUS_PATH), time.monotonic(),
                    time.time(), fusion_blind_search_direction, sensor_recovery_requested,
                    bottom_far_present=bottom_far_present,
                    fusion_style_line=fusion_style_line,
                    virtual_sensors=virtual_sensors,
                )

                line_follower_command = (
                    line_controller.calculate(
                        line_candidate_mask,
                        green_status,
                        maneuver_state.green_direction,
                        maneuver_state.gap_forward_active,
                        virtual_sensors=virtual_sensors,
                        blind_search_requested=gap_blind_search_requested,
                        local_line_lost=gap_validator.decision == "LOST",
                        sensor_recovery_requested=sensor_recovery_requested,
                        fusion_style_line=fusion_style_line,
                        curva_verde_iniciada=maneuver_state.green_curve_started,
                        blind_search_preferred_direction=(
                            fusion_blind_search_direction
                        ),
                        gap_fusion_reacquire_active=(
                            maneuver_state.gap_fusion_reacquire_active
                        ),
                    )
                )
                green_candidate_hold = apply_green_candidate_fusion_hold(
                    line_follower_command,
                    maneuver_state.last_applied_fusion_command,
                    maneuver_state.green_armed,
                    maneuver_state.green_direction,
                    green_status,
                    maneuver_state.green_candidate_hold_frames,
                    maneuver_state.green_candidate_hold_blocked,
                )
                line_follower_command = green_candidate_hold["command"]
                green_candidate_hold_active = green_candidate_hold["active"]
                maneuver_state.green_candidate_hold_frames = green_candidate_hold["frames"]
                maneuver_state.green_candidate_hold_blocked = green_candidate_hold["blocked"]
                near_fine_position = line_follower_command["nearFinePosition"]

                # Confirma que o robô realmente começou a entrar no ramo
                # indicado pelo marcador verde.
                if not maneuver_state.green_curve_started:
                    if (
                        maneuver_state.green_direction == "ESQUERDA"
                        and near_fine_position is not None
                        and near_fine_position <= -LIMIAR_CURVA_VERDE_INICIADA
                    ):
                        maneuver_state.green_curve_started = True
                        maneuver_state.green_centered_frames = 0

                    elif (
                        maneuver_state.green_direction == "DIREITA"
                        and near_fine_position is not None
                        and near_fine_position >= LIMIAR_CURVA_VERDE_INICIADA
                    ):
                        maneuver_state.green_curve_started = True
                        maneuver_state.green_centered_frames = 0

                # Depois que a curva começou, exige alinhamento simultâneo no NEAR
                # e em uma fileira frontal trusted antes de liberar a prioridade.
                if (
                    maneuver_state.green_direction != "NENHUMA"
                    and maneuver_state.green_curve_started
                ):
                    if green_maneuver_is_geometrically_complete(
                        maneuver_state.green_curve_started,
                        near_fine_position,
                        line_follower_command["mediumTrusted"],
                        line_follower_command["mediumPosition"],
                        line_follower_command["farTrusted"],
                        line_follower_command["farPosition"],
                    ):
                        maneuver_state.green_centered_frames += 1
                    else:
                        maneuver_state.green_centered_frames = 0

                    if (
                        maneuver_state.green_centered_frames
                        >= QUADROS_CENTRALIZADO_PARA_CONCLUIR
                    ):
                        completed_green_state = update_green_maneuver_state(
                            maneuver_state.green_direction,
                            maneuver_state.green_active_frames,
                            raw_line_visible,
                            completed=True,
                        )
                        maneuver_state.green_direction = completed_green_state["direction"]
                        maneuver_state.green_active_frames = completed_green_state[
                            "activeFrames"
                        ]
                        maneuver_state.green_curve_started = False
                        maneuver_state.green_centered_frames = 0
                        line_controller.line_search_tracker.stop()

                # O rearme começa somente depois que a manobra deixa de estar ativa.
                maneuver_state.green_armed, maneuver_state.green_clear_frames = update_green_rearm_state(
                    maneuver_state.green_armed,
                    maneuver_state.green_clear_frames,
                    maneuver_state.green_direction,
                    green_status["greenCandidateCount"],
                )
                update_green_control_telemetry(
                    green_status,
                    maneuver_state.green_direction,
                    maneuver_state.green_armed,
                    maneuver_state.green_clear_frames,
                )
                green_status["greenCandidateHoldActive"] = bool(
                    green_candidate_hold_active
                )
                green_status["greenCandidateHoldFrames"] = int(
                    maneuver_state.green_candidate_hold_frames
                )
                if not green_candidate_hold_active:
                    maneuver_state.last_applied_fusion_command = capture_valid_fusion_command(
                        line_follower_command
                    )

                if line_ipc_enabled:
                    if not green_candidate_hold_active and gap_validator.near["present"]:
                        gap_validator.remember(
                            line_candidate_mask, fusion_style_line, line_follower_command,
                            line_timestamp, line_sequence,
                        )
                    line_follower_command.update(gap_validator.diagnostics())

                line_control_ms = (
                    time.perf_counter() - line_control_started
                ) * 1000.0

                line_follower_command["lineProcessingMs"] = (
                    line_vision_ms
                    + geometric_guidance["processingMs"]
                    + line_control_ms
                )

                line_timings["lineProcessingMs"] = (
                    line_follower_command["lineProcessingMs"]
                )

                if line_ipc_enabled:
                    # Somente a CAM0/inferior publica o ponto de extensão 0/0.
                    status_publisher.save_line_status(
                        line_follower_command,
                        line_timestamp,
                        line_sequence,
                        green_status,
                        specular_repair_status=specular_repair_status,
                        silver_status=silver_shadow_status,
                    )

                if green_capture_requested:
                    try:
                        green_mask_stages = create_green_mask_stages(
                            raw_frame,
                            vision_geometry["green_end_y"],
                            camera_format,
                            green_start_y=dead_zone_end_y,
                        )
                        save_green_capture(
                            raw_frame,
                            camera_format,
                            green_mask_stages,
                            green_candidates,
                            green_rejected,
                            green_interpretation,
                            green_capture_metadata,
                        )
                        print(
                            "Captura diagnóstica verde gravada em /dev/shm.",
                            flush=True,
                        )
                    except Exception as error:
                        print(
                            f"Falha na captura diagnóstica verde: {error}",
                            flush=True,
                        )
                    finally:
                        try:
                            os.unlink(GREEN_CAPTURE_REQUEST_PATH)
                        except FileNotFoundError:
                            pass

                # Os pares de calibração usam a máscara real antes de qualquer
                # desenho. A gravação de PNG ocorre fora do loop de processamento.
                calibration_capture.after_frame(
                    raw_frame, line_candidate_mask, structural_mask,
                    green_capture_metadata, line_sequence, fusion_style_line,
                    green_status,
                )
                display_mode = get_display_mode()
                uncorrected_line_candidate_mask = None
                if (
                    display_mode == DISPLAY_MODE_LINE
                    and specular_repair_status.get(
                        "illuminationCorrectionActive",
                        False,
                    )
                ):
                    # A comparação completa custa outra morfologia e só roda
                    # no modo binário de diagnóstico. O stream normal mantém
                    # uma única pipeline e a máscara de controle não é tocada.
                    uncorrected_profile = dict(vision_profile)
                    uncorrected_profile[
                        "line_illumination_correction_enabled"
                    ] = False
                    uncorrected_filtered_mask, uncorrected_roi_start_y = (
                        create_filtered_line_mask(
                            raw_frame,
                            uncorrected_profile,
                            camera_format,
                        )
                    )
                    uncorrected_structural_mask = create_structural_line_mask(
                        uncorrected_filtered_mask,
                        uncorrected_roi_start_y,
                        vision_geometry["structural_end_y"],
                        dead_zone_end_y,
                    )
                    uncorrected_line_candidate_mask = create_line_candidate_mask(
                        uncorrected_structural_mask,
                        uncorrected_profile,
                        green_mask=(
                            green_mask
                            if green_processing_enabled
                            and vision_profile.get("line_exclude_green", False)
                            else None
                        ),
                    )
                frame = create_display_frame(
                    raw_frame,
                    line_candidate_mask,
                    green_mask,
                    roi_start_y,
                    display_mode,
                    structural_mask,
                )

                _illumination_gain, illumination_zone, _illumination_cached = (
                    line_illumination_data(vision_profile, raw_frame.shape)
                )
                draw_line_illumination_overlay(
                    frame,
                    line_candidate_mask,
                    illumination_zone,
                    specular_repair_status,
                    display_mode,
                    uncorrected_line_candidate_mask,
                )

                if camera_profile["role"] == "down":
                    if LEGACY_LINE_DEBUG_ENABLED:
                        draw_line_control_overlay(frame, line_follower_command)
                        draw_virtual_sensor_geometry(
                            frame,
                            line_follower_command,
                            show_debug_details=True,
                        )
                        draw_normal_trajectory_overlay(
                            frame,
                            normal_trajectory,
                        )

                # A câmera inferior mantém o controle e os nove sensores virtuais.
                # As linhas estruturais antigas permanecem nas outras câmeras.
                if camera_profile["role"] != "down":
                    cv2.line(
                        frame,
                        (0, roi_start_y),
                        (frame.shape[1] - 1, roi_start_y),
                        (0, 255, 255),
                        vision_profile["overlay_line_thickness"],
                    )
                    ignored_start_y = vision_geometry["ignored_start_y"]
                    if ignored_start_y is not None:
                        cv2.line(
                            frame,
                            (0, ignored_start_y),
                            (frame.shape[1] - 1, ignored_start_y),
                            (0, 0, 255),
                            vision_profile["overlay_thin_line_thickness"],
                        )

                # Os contornos, símbolos e ROIs verdes continuam visíveis também
                # na câmera inferior sem alterar a classificação ou o controle.
                if green_processing_enabled:
                    interpretation = green_status["greenInterpretation"]
                    green_overlay_accepted = bool(
                        green_status["greenConfirmed"]
                        and interpretation in VISIBLE_GREEN_INTERPRETATIONS
                        and green_raw_interpretation == interpretation
                        and green_status["greenPathBlackValid"]
                    )
                    overlay_interpretation = (
                        interpretation
                        if green_overlay_accepted
                        else green_raw_interpretation
                    )
                    if display_mode == DISPLAY_MODE_LINE:
                        draw_line_mode_green_overlays(
                            frame,
                            green_rejected,
                            "SEM_DECISAO",
                            False,
                        )
                        draw_line_mode_green_overlays(
                            frame,
                            green_candidates,
                            overlay_interpretation,
                            green_overlay_accepted,
                        )
                    else:
                        draw_green_candidate_overlays(
                            frame,
                            green_rejected,
                            "SEM_DECISAO",
                            False,
                        )
                        draw_green_candidate_overlays(
                            frame,
                            green_candidates,
                            overlay_interpretation,
                            green_overlay_accepted,
                        )
                    draw_green_rejection_details(
                        frame,
                        green_rejected,
                    )
                    draw_green_roi_overlays(
                        frame,
                        green_overlay_roi_interpretation,
                    )

                if camera_profile["role"] == "down":
                    # O overlay Fusion é desenhado por último para permanecer legível
                    # mesmo quando a validação de verde também está visível.
                    draw_fusion_style_line_overlay(
                        frame,
                        fusion_style_line,
                        line_follower_command,
                    )
                    draw_near_presence_overlay(frame, gap_validator.near, line_follower_command)
                    # O shadow é desenhado somente na cópia exibida. O frame bruto
                    # usado pela visão, pelo modelo e pelo dataset permanece intacto.
                    draw_silver_shadow_overlay(frame, silver_shadow_status)

                now = time.monotonic()
                elapsed = now - previous_time
                previous_time = now
                if elapsed > 0.0:
                    actual_fps = 1.0 / elapsed
                    smoothed_fps = (
                        smoothed_fps * 0.90 + actual_fps * 0.10
                        if smoothed_fps > 0.0
                        else actual_fps
                    )

                stream_due = stream_frame_is_due(now, last_stream_time)
                snapshot_due = now - last_snapshot_time >= 1.0 / SNAPSHOT_FRAME_FPS
                if stream_due or snapshot_due:
                    jpeg = encode_frame(frame)
                    if jpeg is not None:
                        if stream_due:
                            publish_stream_frame(jpeg)
                            last_stream_time = now
                        if snapshot_due:
                            save_frame(jpeg)
                            last_snapshot_time = now

                if now - last_status_time >= 1.0 / STATUS_FPS:
                    status_publisher.save_status(
                        smoothed_fps,
                        camera_profile,
                        camera_details,
                        camera_format,
                        line_timestamp=line_timestamp,
                        line_sequence=line_sequence,
                        specular_repair_status=specular_repair_status,
                        green_status=green_status,
                        line_timings=line_timings,
                        far_line_confidence=line_follower_command.get(
                            "farLineConfidence",
                            0.0,
                        ),
                        medium_line_confidence=line_follower_command.get(
                            "mediumLineConfidence",
                            0.0,
                        ),
                        far_thickness_consistency=line_follower_command.get(
                            "farThicknessConsistency",
                            0.0,
                        ),
                        medium_thickness_consistency=line_follower_command.get(
                            "mediumThicknessConsistency",
                            0.0,
                        ),
                        normal_trajectory=normal_trajectory,
                        fusion_style_line=fusion_style_line,
                        line_follower_command=line_follower_command,
                        silver_shadow_status=silver_shadow_status,
                    )
                    last_status_time = now
        except Exception as error:
            import traceback
            traceback.print_exc()
            error_message = f"Camera script failed: {error}"
            print(error_message, flush=True)
            status_publisher.save_status(
                0.0,
                camera_profile,
                camera_details,
                active=False,
                error_message=error_message,
            )
            return 1
        finally:
            if calibration_capture is not None:
                calibration_capture.close()
            if stream_server is not None:
                stream_server.shutdown()
                stream_server.server_close()
            if GPIO is not None and light_ready:
                GPIO.output(LIGHT_PIN_BOARD, GPIO.LOW)
                GPIO.cleanup()
            if picam2 is not None and camera_started:
                picam2.stop()

        return 0


def main():
    """Executa a fachada histórica da câmera inferior."""

    return DownwardCameraApplication().run()
