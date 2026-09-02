"""Processa exclusivamente a câmera inferior e publica sua telemetria visual."""

import argparse
import json
import math
import os
import signal
import threading
import time
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import traceback
from urllib.parse import parse_qs, urlsplit


import cv2  # type: ignore
import numpy as np

from vision.green_observation import (
    GREEN_CLEAR_HYSTERESIS_FRAMES,
    GREEN_CONFIRMATION_FRAMES,
    GREEN_DIRECTION_RETENTION_SECONDS,
    GREEN_SINGLE_OBSERVATION_FRAMES,
    GreenObservationTracker,
)
from vision.green_detector import (
    CAMERA_ARRAY_COLOR_ORDER,
    GREEN_ASPECT_RATIO_MAX,
    GREEN_ASPECT_RATIO_MIN,
    GREEN_CAPTURE_CANDIDATES_PATH,
    GREEN_CAPTURE_FINAL_MASK_PATH,
    GREEN_CAPTURE_HSV_MASK_PATH,
    GREEN_CAPTURE_RAW_PATH,
    GREEN_CAPTURE_STATS_PATH,
    GREEN_CLOSE_ITERATIONS,
    GREEN_CLOSE_KERNEL_SIZE,
    GREEN_FRAGMENT_MERGE_DISTANCE_PX,
    GREEN_HUE_MAX,
    GREEN_HUE_MIN,
    GREEN_INTERPRETATIONS,
    GREEN_MIN_AREA_PX,
    GREEN_MIN_AREA_RATIO,
    GREEN_MIN_DIMENSION_PX,
    GREEN_MIN_EXTENT,
    GREEN_OPEN_ITERATIONS,
    GREEN_OPEN_KERNEL_SIZE,
    GREEN_OBSERVATION_STATES,
    GREEN_PAIR_MAX_VERTICAL_DISTANCE_HEIGHTS,
    GREEN_PARTIAL_AREA_FACTOR,
    GREEN_PARTIAL_ASPECT_RATIO_MIN,
    GREEN_PARTIAL_BORDER_TOLERANCE_PX,
    GREEN_PARTIAL_DIMENSION_FACTOR,
    GREEN_PARTIAL_EXTENT_MIN,
    GREEN_ROI_HALF_SIZE_DIVISOR,
    GREEN_ROI_MIN_BLACK_RATIO,
    GREEN_ROI_MIN_VISIBLE_RATIO,
    GREEN_SATURATION_MIN,
    GREEN_VALUE_MIN,
    VISIBLE_GREEN_INTERPRETATIONS,
    analyze_green_marker_contours,
    build_green_capture_stats,
    build_green_status,
    camera_array_rgb_channels,
    channel_percentiles,
    component_pixel_statistics,
    contour_touches_useful_border,
    create_green_mask,
    create_green_mask_stages,
    describe_green_contour,
    empty_green_status,
    expanded_boxes_overlap,
    find_green_candidates,
    frame_to_hsv,
    green_geometry_is_valid,
    green_geometry_rejection_reasons,
    green_marker_roi_geometry,
    green_minimum_area,
    green_observation_state,
    group_fragment_boxes,
    is_hsv_green,
    json_safe_camera_metadata,
    mask_active_percent,
    measure_black_roi,
    measure_horizontal_black_roi,
    merge_green_fragments,
    rgb_pixel_to_camera_array,
    save_green_capture,
)
from vision.geometry import (
    GAP_NEAR_HISTORY_FRAMES,
    GEOMETRIC_GAP_CONFIRM_OFFSET_PX,
    GEOMETRIC_GAP_FORWARD_MAX_FRAMES,
    GEOMETRIC_GAP_MAX_CENTER_SHIFT_PX,
    GEOMETRIC_GAP_MAX_SEARCH_PX,
    GEOMETRIC_GAP_PROJECTION_POINTS,
    GEOMETRIC_GAP_REACQUIRE_FRAMES,
    GEOMETRIC_GAP_SEARCH_STEP_PX,
    GEOMETRIC_PATH_BAND_HALF_HEIGHT,
    GEOMETRIC_PATH_FAR_Y_RATIO,
    GEOMETRIC_PATH_LOCAL_HEADING_POINTS,
    GEOMETRIC_PATH_NEAR_Y_RATIO,
    GEOMETRIC_TRACE_EXIT_MARGIN_PX,
    GEOMETRIC_TRACE_MAX_BACKTRACK_Y_PX,
    GEOMETRIC_TRACE_MAX_POINTS,
    GEOMETRIC_TRACE_MAX_TURN_DEG,
    GEOMETRIC_TRACE_SEARCH_RANGE_PX,
    GEOMETRIC_TRACE_STEP_PX,
    GEOMETRIC_TRACE_TARGET_PROGRESS_GAIN,
    calculate_geometric_far_heading,
    estimate_geometric_initial_direction,
    extract_gap_geometric_guidance,
    extract_geometric_line_path,
    find_active_band_segments,
    find_geometric_gap_start,
    find_geometric_lateral_exit,
    find_next_geometric_path_point,
    project_geometric_gap_to_near,
)
from vision.line_control import (
    GREEN_MANEUVER_TIMEOUT_FRAMES,
    LIMIAR_CENTRALIZACAO_VERDE,
    LIMIAR_CURVA_VERDE_INICIADA,
    NORMAL_BASE_POWER,
    NORMAL_FULL_STEERING_ERROR,
    NORMAL_INNER_MIN_POWER,
    NORMAL_MAX_POWER,
    QUADROS_CENTRALIZADO_PARA_CONCLUIR,
    QUADROS_PARA_REARMAR_VERDE,
    VIRTUAL_MEDIUM_STRONG_THRESHOLD,
    calculate_line_follower_command,
    gap_entry_is_required,
    green_direction_to_search_direction,
    map_normal_steering_error,
    update_gap_forward_recovery,
    update_gap_recent_near_frames,
    update_green_maneuver_state,
    virtual_medium_scan_direction,
    virtual_raw_line_is_visible,
    virtual_recovery_sensor_direction,
)
from vision.search import (
    VIRTUAL_BLIND_SEARCH_INITIAL_FRAMES,
    VIRTUAL_BLIND_SEARCH_REVERSE_FRAMES,
    VirtualLineSearchTracker,
)
from vision.medium_spin import (
    VIRTUAL_HARD_CORNER_FAR_RECOVERY_FRAMES,
    VIRTUAL_HARD_CORNER_MAX_FRAMES,
    VIRTUAL_HARD_CORNER_MEDIUM_RECOVERY_THRESHOLD,
    VIRTUAL_HARD_CORNER_NEAR_RECOVERY_THRESHOLD,
    VIRTUAL_HARD_CORNER_RECOVERY_FRAMES,
    VIRTUAL_MEDIUM_CRITICAL_INVALID_MAX_FRAMES,
    VIRTUAL_MEDIUM_DIRECTION_LOST_THRESHOLD,
    VIRTUAL_MEDIUM_SPIN_ENTER_THRESHOLD,
    VIRTUAL_MEDIUM_SPIN_EXIT_THRESHOLD,
    VIRTUAL_MEDIUM_SPIN_POWER,
    VirtualMediumSpinTracker,
)
from vision.numeric import (
    DOWNWARD_REFERENCE_FRAME_HEIGHT,
    finite_virtual_position,
    scaled_odd_kernel_size,
    scaled_reference_pixels,
)
from vision.pivot import (
    PIVOT_ENTER_THRESHOLD,
    PIVOT_EXIT_THRESHOLD,
    PIVOT_STATE_LEFT,
    PIVOT_STATE_NONE,
    PIVOT_STATE_RIGHT,
    VirtualPivotStateTracker,
)
from vision.reorient import (
    VIRTUAL_MEDIUM_SCAN_MAX_FRAMES,
    VIRTUAL_REORIENT_CONFIRMATION_FRAMES,
    VIRTUAL_REORIENT_DIRECTION_THRESHOLD,
    VIRTUAL_REORIENT_RECOVERY_FRAMES,
    VIRTUAL_STATE_NORMAL,
    VIRTUAL_STATE_REORIENT_LEFT,
    VIRTUAL_STATE_REORIENT_RIGHT,
    VirtualTurnStateTracker,
    virtual_reorient_direction,
    virtual_sensor_trust_is_active,
)
from vision.virtual_sensors import (
    FAR_TRUST_MIN_CONFIDENCE,
    FAR_TRUST_MIN_THICKNESS_PX,
    LINE_TRUST_ABSOLUTE_THIN_VETO_PX,
    MEDIUM_TRUST_MIN_CONFIDENCE,
    MEDIUM_TRUST_MIN_THICKNESS_PX,
    VIRTUAL_CENTER_X0,
    VIRTUAL_CENTER_X1,
    VIRTUAL_FAR_BAND_Y0,
    VIRTUAL_FAR_BAND_Y1,
    VIRTUAL_FAR_CENTER_X0,
    VIRTUAL_FAR_CENTER_X1,
    VIRTUAL_FAR_LEFT_X0,
    VIRTUAL_FAR_LEFT_X1,
    VIRTUAL_FAR_RIGHT_X0,
    VIRTUAL_FAR_RIGHT_X1,
    VIRTUAL_FAR_Y0,
    VIRTUAL_FAR_Y1,
    VIRTUAL_FINE_CENTER_DEADBAND,
    VIRTUAL_FINE_CENTER_GAIN,
    VIRTUAL_FINE_CENTER_MAX_CORRECTION,
    VIRTUAL_HEADING_FULL_SCALE_DEG,
    VIRTUAL_HEADING_GAIN,
    VIRTUAL_LEFT_X0,
    VIRTUAL_LEFT_X1,
    VIRTUAL_LINE_CONFIDENCE_AREA_WEIGHT,
    VIRTUAL_LINE_CONFIDENCE_CONSISTENCY_WEIGHT,
    VIRTUAL_LINE_CONFIDENCE_CONTINUITY_WEIGHT,
    VIRTUAL_LINE_CONFIDENCE_REFERENCE_HEIGHT_PX,
    VIRTUAL_LINE_CONFIDENCE_REFERENCE_WIDTH_PX,
    VIRTUAL_LINE_CONFIDENCE_SAMPLE_STEP_PX,
    VIRTUAL_LINE_CONFIDENCE_THICKNESS_WEIGHT,
    VIRTUAL_LINE_CONSISTENCY_MAX_RELATIVE_DISPERSION,
    VIRTUAL_LINE_EXPECTED_THICKNESS_BOTTOM_PX,
    VIRTUAL_LINE_EXPECTED_THICKNESS_TOP_PX,
    VIRTUAL_LINE_THICKNESS_CORE_PERCENTILE,
    VIRTUAL_MEDIUM_CENTER_X0,
    VIRTUAL_MEDIUM_CENTER_X1,
    VIRTUAL_MEDIUM_LEFT_X0,
    VIRTUAL_MEDIUM_LEFT_X1,
    VIRTUAL_MEDIUM_RIGHT_X0,
    VIRTUAL_MEDIUM_RIGHT_X1,
    VIRTUAL_MEDIUM_SCAN_POSITION_THRESHOLD,
    VIRTUAL_MEDIUM_WING_Y0,
    VIRTUAL_MEDIUM_WING_Y1,
    VIRTUAL_MEDIUM_Y0,
    VIRTUAL_MEDIUM_Y1,
    VIRTUAL_NEAR_Y0,
    VIRTUAL_NEAR_Y1,
    VIRTUAL_RIGHT_X0,
    VIRTUAL_RIGHT_X1,
    VIRTUAL_ROW_MIN_ACTIVATION,
    apply_virtual_fine_center_deadband,
    calculate_virtual_heading_angle,
    calculate_virtual_near_fine_position,
    calculate_virtual_row_line_confidence,
    calculate_virtual_row_position,
    calculate_virtual_steering_error,
    component_labels_in_row,
    create_virtual_row_component_mask,
    draw_virtual_sensor_geometry,
    empty_virtual_row_line_measurement,
    expected_virtual_line_thickness_px,
    line_measurement_is_trusted,
    measure_component_thickness,
    measure_virtual_row_line_confidence,
    non_negative_line_measurement,
    normalized_line_confidence,
    normalized_thickness_consistency,
    read_virtual_line_sensors,
    read_virtual_sensor,
    resolve_virtual_sensor_geometry,
    robust_component_thickness_px,
    select_virtual_trust_row_geometry,
    virtual_fine_position_to_point,
    virtual_row_position_to_point,
    virtual_sensor_is_active,
    virtual_sensor_regions,
)

try:
    import RPi.GPIO as GPIO  # type: ignore
except ImportError:
    GPIO = None

try:
    from libcamera import Transform  # type: ignore[import]
    from picamera2 import Picamera2  # type: ignore[import]
except ImportError:
    Transform = None
    Picamera2 = None


FRAME_PATH = "/tmp/obr_camera_frame.jpg"
TEMP_FRAME_PATH = "/tmp/obr_camera_frame.tmp.jpg"
STATUS_PATH = "/tmp/obr_camera_status.json"
TEMP_STATUS_PATH = "/tmp/obr_camera_status.tmp.json"
LINE_STATUS_PATH = "/dev/shm/obr_line_status.json"
TEMP_LINE_STATUS_PATH = "/dev/shm/obr_line_status.tmp.json"
GREEN_CAPTURE_REQUEST_PATH = "/dev/shm/obr_green_capture_request"

LIGHT_PIN_BOARD = 40
MJPEG_STREAM_PORT = 8090
MJPEG_STREAM_PATH = "/stream.mjpg"
MJPEG_STREAM_FPS = 30
SNAPSHOT_FRAME_FPS = 2
STATUS_FPS = 5
JPEG_QUALITY = 82
CAMERA_PIXEL_FORMATS = ("RGB888",)

DEFAULT_CAMERA_INDICES = {
    "down": 0,
    "forward": 1,
}
CAMERA_INDEX_ENVIRONMENT = {
    "down": "OBR_DOWNWARD_CAMERA_INDEX",
    "forward": "OBR_FORWARD_CAMERA_INDEX",
}
DISPLAY_MODE_REAL = "real"
DISPLAY_MODE_LINE = "line"
DISPLAY_MODE_GREEN = "green"
DISPLAY_MODES = (
    DISPLAY_MODE_REAL,
    DISPLAY_MODE_LINE,
)


def environment_flag(name, default):
    """Lê uma flag booleana de ambiente sem aceitar valores ambíguos."""

    value = os.environ.get(name)
    if value is None:
        return bool(default)
    normalized = value.strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise ValueError(f"{name} deve ser 0/1, true/false, yes/no ou on/off.")


def configured_camera_indices(environment=None):
    """Lê os dois índices físicos sem permitir fallback ou papéis duplicados."""

    environment = os.environ if environment is None else environment
    indices = {}
    for role, variable_name in CAMERA_INDEX_ENVIRONMENT.items():
        raw_value = environment.get(
            variable_name,
            str(DEFAULT_CAMERA_INDICES[role]),
        )
        try:
            camera_index = int(raw_value)
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"{variable_name} deve ser um índice inteiro não negativo."
            ) from error
        if camera_index < 0:
            raise ValueError(
                f"{variable_name} deve ser um índice inteiro não negativo."
            )
        indices[role] = camera_index

    if indices["down"] == indices["forward"]:
        raise ValueError(
            "As câmeras inferior e frontal não podem usar o mesmo índice."
        )
    return indices


def camera_number(camera_info, fallback_index):
    """Obtém o número enumerado sem deduzir o papel pelo modelo do sensor."""

    value = camera_info.get("Num", fallback_index)
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(fallback_index)


def resolve_camera_assignments(camera_infos, camera_indices):
    """Associa índices configurados aos papéis sem trocar câmeras ausentes."""

    enumerated = {
        camera_number(camera_info, fallback_index): camera_info
        for fallback_index, camera_info in enumerate(camera_infos)
    }
    return {
        role: {
            "role": role,
            "index": camera_index,
            "available": camera_index in enumerated,
            "info": enumerated.get(camera_index),
        }
        for role, camera_index in camera_indices.items()
    }


def require_camera_assignment(assignments, role):
    """Falha claramente se a câmera exigida para o papel não foi enumerada."""

    assignment = assignments[role]
    if not assignment["available"]:
        raise RuntimeError(
            f"Câmera {role} configurada no índice {assignment['index']} "
            "não foi encontrada; não haverá troca automática de papel."
        )
    return assignment


def camera_role_publishes_line_status(role):
    """Restringe o IPC do segue-faixa ao papel físico da câmera inferior."""

    return role == "down"


def log_camera_inventory(camera_infos, assignments):
    """Registra inventário, identificador e papel configurado de cada câmera."""

    if not camera_infos:
        print("Nenhuma câmera foi enumerada pelo Picamera2.", flush=True)
    for fallback_index, camera_info in enumerate(camera_infos):
        camera_index = camera_number(camera_info, fallback_index)
        print(
            "Câmera enumerada: "
            f"índice={camera_index}, modelo={camera_info.get('Model', '')}, "
            f"identificador={camera_info.get('Id', '')}, "
            f"localização={camera_info.get('Location', '')}, "
            f"rotação={camera_info.get('Rotation', '')}.",
            flush=True,
        )
    for role in ("down", "forward"):
        assignment = assignments[role]
        availability = "disponível" if assignment["available"] else "indisponível"
        print(
            f"Papel configurado: {role}=índice {assignment['index']} "
            f"({availability}).",
            flush=True,
        )


# Esta chave permite desativar apenas o diagnóstico verde sem alterar a câmera.
GREEN_PROCESSING_ENABLED = environment_flag("GREEN_PROCESSING_ENABLED", True)

# O detector de referência exige mais de 3.000 pixels verdes em 320×200.
# Este limite aceita o marcador oficial observado na pista, enquanto os filtros
# de HSV, formato e associação com a faixa continuam bloqueando falsos verdes.
LINE_MIN_COMPONENT_AREA_PX = 120
LINE_MIN_COMPONENT_THICKNESS_PX = 11.0
LINE_MIN_COMPONENT_CORE_RATIO = 0.15

# O LED físico pode criar pequenos reflexos brancos dentro da fita preta. Este
# reparo atua somente em ilhas claras completamente cercadas pela máscara preta;
# jamais fecha uma abertura ligada ao fundo, pois ela pode ser uma interrupção real.
SPECULAR_REPAIR_REFERENCE_FRAME_HEIGHT = 360.0
SPECULAR_REPAIR_MAX_DIAMETER_PX = 12.0
SPECULAR_REPAIR_MIN_VALUE = 180
SPECULAR_REPAIR_MAX_SATURATION = 60


# A câmera inferior corrige sua montagem pelo Picamera2. A frontal mantém a
# captura neutra e aplica sua rotação no próprio processo, pois a OV5647 não
# entregou o flip de captura de forma consistente nos testes reais.
# Manter a transformação no Picamera2 evita rotacionar cada frame no OpenCV.
CAMERA_ROTATION_DEGREES = 180

# Ajustes básicos de imagem. Eles afetam somente a visualização e não geram
# qualquer decisão de movimento.
CAMERA_SHARPNESS = 1.2
CAMERA_CONTRAST = 1.05
CAMERA_SATURATION = 1.0
CAMERA_EXPOSURE_VALUE = 0.4

# Cada papel define de forma independente a captura e os parâmetros visuais.
# O perfil inferior não herda ROIs nem limites em pixels da câmera frontal.
CAMERA_PROFILES = {
    "forward": {
        "role": "forward",
        "rotation_degrees": 0,
        "main_size": (960, 540),
        "sensor_size": (1920, 1080),
        "sensor_bit_depth": 10,
        "target_fps": 30,
        "vision": {
            "line_roi_start_ratio": 0.0,
            "line_threshold": 100,
            "open_kernel_size": 3,
            "close_kernel_size": 5,
            "full_line_min_short_side_ratio": 50.0 / 540.0,
            "overlay_line_thickness": 2,
            "overlay_thin_line_thickness": 1,
            "debug_text_overlay": False,
        },
    },
    "down": {
        "role": "down",
        "rotation_degrees": CAMERA_ROTATION_DEGREES,
        "main_size": (480, 360),
        "sensor_size": (1640, 1232),
        "sensor_bit_depth": 10,
        "target_fps": 30,
        "vision": {
            "line_roi_start_ratio": 0.0,
            # A câmera inferior perdeu a iluminação dedicada. O fechamento
            # grande estima a claridade do piso ao redor da fita e evita que
            # papel branco sombreado seja classificado como linha preta.
            # Valor de referência em 640×480. A criação da máscara escala para
            # a altura recebida: em 480×360, 201 vira 151.
            "line_background_kernel_size": 201,
            # Um pixel precisa estar pelo menos 32% abaixo do fundo local.
            # Aumentar este valor aceita linhas com menos contraste, mas também
            # aumenta o risco de aceitar sombras como parte da linha.
            "line_max_background_ratio_percent": 70,
            # Mesmo com contraste local, tons acima deste limite não são pretos.
            # A unidade é o nível de cinza de 8 bits, entre 0 e 255. Aumentar o
            # limite aceita sombras; reduzir demais pode perder uma fita clara.
            "line_max_brightness": 190,
            # Valores de referência em 640×480; são sempre escalados para
            # kernels ímpares antes da morfologia (17 vira 13 e 7 vira 5).
            "open_kernel_shape": "ellipse",
            "open_kernel_size": 17,
            # O fechamento 7×7 preenche pequenas falhas sem unir objetos
            # separados à linha de 2 cm.
            "close_kernel_size": 11,
            # Vinte pixels mantêm aproximadamente a mesma espessura angular
            # mínima do perfil frontal após o aumento de campo de visão.
            "full_line_min_short_side_ratio": 20.0 / 480.0,
            # O perfil inferior preserva componentes amplos: cruzamentos e
            # curvas próximas podem ocupar quase toda a máscara útil.
            "full_line_max_area_ratio": 1.0,
            # As coordenadas usam o frame de referência 640×480 validado.
            # A conversão centralizada mantém a mesma geometria proporcional se
            # a altura real do frame for diferente durante um diagnóstico.
            "geometry_reference": {
                "frame_height": 480,
                "structural_end_y": 480,
                # O verde mantém a ROI vertical usada em sua calibração.
                # Ampliar a máscara preta não deve mudar seus filtros.
                "green_end_y": 400,
            },
            "green_detection_enabled": True,
            "overlay_line_thickness": 2,
            "overlay_thin_line_thickness": 1,
            "debug_text_overlay": False,
        },
    },
}

running = True
latest_jpeg = None
latest_jpeg_sequence = 0
active_stream_clients = 0
frame_condition = threading.Condition()
selected_display_mode = DISPLAY_MODE_REAL
display_mode_lock = threading.Lock()


def normalize_display_mode(value):
    """Mantém o modo visual dentro das duas opções aceitas pelo dashboard."""

    normalized = str(value or "").strip().lower()
    return normalized if normalized in DISPLAY_MODES else DISPLAY_MODE_REAL


def set_display_mode(value):
    """Seleciona apenas a imagem codificada, sem alterar o processamento visual."""

    global selected_display_mode
    normalized = normalize_display_mode(value)
    with display_mode_lock:
        selected_display_mode = normalized
    return normalized


def get_display_mode():
    """Lê o modo visual solicitado pela conexão MJPEG ativa."""

    with display_mode_lock:
        return selected_display_mode


def create_display_frame(
    raw_frame,
    line_candidate_mask,
    green_mask,
    roi_start_y,
    display_mode,
    structural_mask=None,
):
    """Cria a base exibida sem modificar o frame ou as máscaras da visão."""

    normalized_mode = normalize_display_mode(display_mode)
    if normalized_mode == DISPLAY_MODE_REAL:
        return raw_frame.copy()

    display_frame = np.zeros_like(raw_frame)
    if normalized_mode == DISPLAY_MODE_LINE:
        line_roi = display_frame[roi_start_y:raw_frame.shape[0], :]
        if structural_mask is not None:
            # Cinza mostra preto segmentado, mas rejeitado pelos filtros
            # estruturais. Branco fica reservado à máscara pronta para o
            # futuro seguidor por sensores virtuais.
            line_roi[structural_mask > 0] = (96, 96, 96)
        line_roi[line_candidate_mask > 0] = (255, 255, 255)
        # A máscara HSV verde aparece junto da faixa preta para depuração sem
        # exigir uma segunda aba nem reativar textos sobre o vídeo.
        useful_green_region = display_frame[:green_mask.shape[0], :]
        useful_green_region[green_mask > 0] = (0, 255, 0)
    return display_frame


def draw_green_decision_symbol(display_frame, center, interpretation, accepted):
    """Desenha símbolos geométricos leves, sem renderizar texto no vídeo."""

    color = (64, 255, 96) if accepted else (0, 220, 255)
    center_x, center_y = center
    if interpretation == "RETORNO_180":
        cv2.ellipse(
            display_frame,
            center,
            (10, 10),
            0,
            25,
            330,
            color,
            1,
            cv2.LINE_AA,
        )
        cv2.arrowedLine(
            display_frame,
            (center_x + 7, center_y - 7),
            (center_x + 11, center_y - 2),
            color,
            1,
            cv2.LINE_AA,
            tipLength=0.45,
        )
    elif interpretation in ("ESQUERDA", "DIREITA"):
        direction = -1 if interpretation == "ESQUERDA" else 1
        cv2.arrowedLine(
            display_frame,
            (center_x, center_y + 10),
            (center_x + direction * 14, center_y + 10),
            color,
            1,
            cv2.LINE_AA,
            tipLength=0.40,
        )

    if accepted:
        cv2.line(
            display_frame,
            (center_x - 7, center_y - 7),
            (center_x - 3, center_y - 3),
            color,
            2,
            cv2.LINE_AA,
        )
        cv2.line(
            display_frame,
            (center_x - 3, center_y - 3),
            (center_x + 6, center_y - 12),
            color,
            2,
            cv2.LINE_AA,
        )
    else:
        diamond = np.asarray([
            (center_x, center_y - 6),
            (center_x + 6, center_y),
            (center_x, center_y + 6),
            (center_x - 6, center_y),
        ], dtype=np.int32)
        cv2.polylines(display_frame, [diamond], True, color, 1, cv2.LINE_AA)


def draw_green_candidate_overlays(
    display_frame,
    candidates,
    interpretation,
    accepted,
):
    """Desenha o candidato forte ou a decisão verde já aceita."""

    color = (64, 255, 96) if accepted else (0, 220, 255)
    for candidate in candidates:
        center = tuple(int(round(value)) for value in candidate["centroid"])
        cv2.drawContours(
            display_frame,
            [candidate["contour"]],
            -1,
            color,
            1,
        )
        cv2.circle(display_frame, center, 3, color, -1)
        draw_green_decision_symbol(
            display_frame,
            center,
            interpretation,
            accepted,
        )


def draw_line_mode_green_overlays(
    display_frame,
    candidates,
    interpretation,
    accepted,
):
    """Desenha na máscara de linha o candidato forte ou o verde aceito."""

    for candidate in candidates:
        color = (0, 255, 0) if accepted else (0, 220, 255)
        cv2.drawContours(
            display_frame,
            [candidate["contour"]],
            -1,
            color,
            2,
        )
        center = tuple(int(round(value)) for value in candidate["centroid"])
        cv2.circle(display_frame, center, 3, color, -1)
        draw_green_decision_symbol(
            display_frame,
            center,
            interpretation,
            accepted,
        )


def draw_green_roi_overlays(display_frame, roi_interpretation):
    """Desenha as duas ROIs perpendiculares usadas na classificação."""

    for marker in roi_interpretation.get("markers", []):
        geometry = marker.get("geometry", {})
        upper = marker.get("upper", {})
        horizontal = marker.get("horizontal", {})
        # A ROI superior confirma a associação com a faixa. A horizontal
        # atravessa os dois lados do marcador e localiza a faixa preta.
        roi_entries = (
            (geometry.get("upper_roi"), upper),
            (geometry.get("horizontal_roi"), horizontal),
        )

        for roi, measurement in roi_entries:
            if roi is None:
                continue
            x1, y1, x2, y2 = (int(value) for value in roi)
            if not measurement.get("measured", False):
                color = (160, 160, 160)
            elif measurement.get("valid", False):
                color = (64, 255, 96)
            else:
                color = (0, 220, 255)
            cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 1)


def draw_green_rejection_details(display_frame, rejected_candidates):
    """Mostra por que um componente verde não chegou à classificação."""

    for candidate in rejected_candidates:
        reasons = green_geometry_rejection_reasons(candidate)
        if "area_below_scaled_reference" in reasons:
            detail = (
                f"GREEN AREA {candidate['area']:.0f}/"
                f"{candidate['scaled_minimum_area']:.0f}"
            )
        else:
            detail = "GREEN REJECT " + (
                ",".join(reasons) if reasons else "UNKNOWN"
            )
        x, y, _width, _height = candidate["bounding_box"]
        cv2.putText(
            display_frame,
            detail,
            (max(4, int(x)), max(14, int(y) - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 220, 255),
            1,
            cv2.LINE_AA,
        )


def parse_camera_profile(arguments=None):
    """Aceita somente a CAM0; a frontal possui um processo dedicado e leve."""

    environment_role = os.environ.get("OBR_CAMERA_ROLE", "down").strip().lower()
    parser = argparse.ArgumentParser(
        description="Captura e processa a câmera inferior do robô."
    )
    parser.add_argument(
        "--camera-role",
        choices=("down",),
        default=environment_role,
        help="Papel físico desta captura: somente down.",
    )
    parsed = parser.parse_args(arguments)
    if parsed.camera_role != "down":
        parser.error(
            "OBR_CAMERA_ROLE deve ser 'down'; use forward_camera_stream.py "
            "para a câmera frontal."
        )
    return CAMERA_PROFILES["down"]


def handle_signal(signum, frame):
    """Encerra o stream de forma limpa quando o serviço recebe um sinal."""

    del signum, frame
    global running
    running = False
    with frame_condition:
        frame_condition.notify_all()


def register_stream_client():
    """Registra uma conexão MJPEG para habilitar a codificação rápida."""

    global active_stream_clients
    with frame_condition:
        active_stream_clients += 1


def unregister_stream_client():
    """Remove uma conexão MJPEG encerrada sem permitir contagem negativa."""

    global active_stream_clients
    with frame_condition:
        active_stream_clients = max(0, active_stream_clients - 1)


def stream_frame_is_due(now, last_stream_time):
    """Solicita JPEG de stream somente enquanto existe cliente conectado."""

    with frame_condition:
        has_clients = active_stream_clients > 0
    return (
        has_clients
        and now - last_stream_time >= 1.0 / MJPEG_STREAM_FPS
    )


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


class CameraStreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format_text, *args):
        del format_text, args

    def do_GET(self):
        request_url = urlsplit(self.path)
        path = request_url.path
        if path != MJPEG_STREAM_PATH:
            self.send_response(404)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(b"Not found")
            return

        query = parse_qs(request_url.query)
        set_display_mode(query.get("mode", [DISPLAY_MODE_REAL])[0])

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        last_sequence = -1
        register_stream_client()
        try:
            while running:
                with frame_condition:
                    frame_condition.wait_for(
                        lambda: latest_jpeg_sequence != last_sequence or not running,
                        timeout=1.0,
                    )
                    if latest_jpeg is None:
                        continue
                    jpeg = latest_jpeg
                    last_sequence = latest_jpeg_sequence

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return
        finally:
            unregister_stream_client()

    def do_HEAD(self):
        path = self.path.split("?", 1)[0]
        if path != MJPEG_STREAM_PATH:
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()


def start_stream_server():
    """Inicia o servidor local usado pelo proxy do dashboard."""

    server = ReusableThreadingHTTPServer(("127.0.0.1", MJPEG_STREAM_PORT), CameraStreamHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def rectangle_values(rectangle):
    """Converte um Rectangle do libcamera em valores simples para o status."""

    if not hasattr(rectangle, "x"):
        x, y, width, height = rectangle
        return {
            "x": int(x),
            "y": int(y),
            "width": int(width),
            "height": int(height),
        }
    return {
        "x": int(rectangle.x),
        "y": int(rectangle.y),
        "width": int(rectangle.width),
        "height": int(rectangle.height),
    }


def camera_transform_settings(camera_profile):
    """Define a transformação física de cada papel sem compartilhar rotação."""

    rotation_degrees = int(camera_profile.get("rotation_degrees", 0))
    if rotation_degrees == 0:
        return {
            "rotation_degrees": 0,
            "hflip": False,
            "vflip": False,
            "name": "identity",
        }
    if rotation_degrees == 180:
        return {
            "rotation_degrees": 180,
            "hflip": True,
            "vflip": True,
            "name": "hvflip",
        }
    raise ValueError(
        "A rotação da câmera deve ser 0 ou 180 graus para esta montagem."
    )


def camera_runtime_details(picam2, camera_profile, camera_config, camera_index):
    """Registra a câmera e o modo físico realmente aceitos pelo Picamera2."""

    sensor_config = camera_config["sensor"]
    sensor_size = tuple(int(value) for value in sensor_config["output_size"])
    requested_sensor_size = tuple(camera_profile["sensor_size"])
    requested_bit_depth = int(camera_profile["sensor_bit_depth"])
    sensor_bit_depth = int(sensor_config["bit_depth"])
    if sensor_size != requested_sensor_size or sensor_bit_depth != requested_bit_depth:
        raise RuntimeError(
            "O Picamera2 não aplicou o modo físico solicitado: "
            f"esperado={requested_sensor_size}/{requested_bit_depth}-bit, "
            f"aplicado={sensor_size}/{sensor_bit_depth}-bit."
        )

    camera = getattr(picam2, "camera", None)
    camera_id = str(getattr(camera, "id", ""))
    properties = getattr(picam2, "camera_properties", {})
    transform_settings = camera_transform_settings(camera_profile)
    return {
        "cameraIndex": int(camera_index),
        "cameraId": camera_id,
        "cameraModel": str(properties.get("Model", "")),
        "sensorMode": {
            "width": sensor_size[0],
            "height": sensor_size[1],
            "bitDepth": sensor_bit_depth,
            "format": str(camera_config["raw"]["format"]),
        },
        "scalerCrop": None,
        "rotationDegrees": transform_settings["rotation_degrees"],
        "transform": transform_settings["name"],
    }


def create_camera(camera_profile, camera_index):
    """Configura a Camera V2 e exige o modo físico definido para seu papel."""

    picam2 = Picamera2(camera_index)
    frame_width, frame_height = camera_profile["main_size"]
    target_fps = camera_profile["target_fps"]
    frame_duration_us = int(1_000_000 / target_fps)
    transform_settings = camera_transform_settings(camera_profile)
    camera_transform = Transform(
        hflip=transform_settings["hflip"],
        vflip=transform_settings["vflip"],
    )
    sensor = {
        "output_size": camera_profile["sensor_size"],
        "bit_depth": camera_profile["sensor_bit_depth"],
    }

    for pixel_format in CAMERA_PIXEL_FORMATS:
        try:
            camera_config = picam2.create_video_configuration(
                main={"size": (frame_width, frame_height), "format": pixel_format},
                sensor=sensor,
                controls={"FrameDurationLimits": (frame_duration_us, frame_duration_us)},
                transform=camera_transform,
                buffer_count=4,
            )
            picam2.configure(camera_config)
            applied_config = picam2.camera_configuration()
            runtime_details = camera_runtime_details(
                picam2,
                camera_profile,
                applied_config,
                camera_index,
            )
            print(
                f"Câmera {camera_profile['role']} configurada em {pixel_format}, "
                f"main {frame_width}x{frame_height}, sensor "
                f"{sensor['output_size'][0]}x{sensor['output_size'][1]} "
                f"{sensor['bit_depth']}-bit e alvo de {target_fps} FPS.",
                flush=True,
            )
            return picam2, pixel_format, runtime_details
        except Exception as error:
            print(f"Configuração {pixel_format} falhou: {error}", flush=True)

    camera_config = picam2.create_still_configuration(
        {"size": (frame_width, frame_height), "format": "RGB888"},
        sensor=sensor,
        transform=camera_transform,
    )
    picam2.configure(camera_config)
    applied_config = picam2.camera_configuration()
    runtime_details = camera_runtime_details(
        picam2,
        camera_profile,
        applied_config,
        camera_index,
    )
    print(
        f"Câmera {camera_profile['role']} configurada em modo still como fallback.",
        flush=True,
    )
    return picam2, "RGB888", runtime_details


def tune_camera_image(picam2):
    """Aplica somente controles visuais suportados pela câmera instalada."""

    controls = {
        "AeEnable": True,
        "AwbEnable": True,
        "ExposureValue": CAMERA_EXPOSURE_VALUE,
        "Sharpness": CAMERA_SHARPNESS,
        "Contrast": CAMERA_CONTRAST,
        "Saturation": CAMERA_SATURATION,
    }
    for name, value in controls.items():
        try:
            picam2.set_controls({name: value})
        except Exception as error:
            print(f"Controle de câmera {name} não foi aplicado: {error}", flush=True)


@lru_cache(maxsize=16)
def cached_structuring_element(shape, width, height):
    """Reutiliza até 16 kernels imutáveis definidos apenas por sua geometria."""

    return cv2.getStructuringElement(shape, (width, height))


def create_line_binary_mask(gray_roi, vision_profile, timings=None):
    """Separa a fita preta usando o método configurado para cada câmera."""

    if timings is not None:
        timings["backgroundKernelMs"] = 0.0
        timings["backgroundCloseMs"] = 0.0
        timings["binaryCompareMs"] = 0.0
    background_kernel_size = vision_profile.get("line_background_kernel_size")
    if background_kernel_size is None:
        _, binary_mask = cv2.threshold(
            gray_roi,
            vision_profile["line_threshold"],
            255,
            cv2.THRESH_BINARY_INV,
        )
        return binary_mask

    # O fechamento grande remove estruturas escuras menores que o kernel e
    # produz uma estimativa da iluminação do piso. A comparação relativa
    # continua funcionando quando o papel branco fica escuro sem o LED.
    background_kernel_started = (
        time.perf_counter() if timings is not None else 0.0
    )
    background_kernel = cached_structuring_element(
        cv2.MORPH_RECT,
        background_kernel_size,
        background_kernel_size,
    )
    if timings is not None:
        timings["backgroundKernelMs"] = (
            time.perf_counter() - background_kernel_started
        ) * 1000.0
    background_close_started = (
        time.perf_counter() if timings is not None else 0.0
    )
    local_background = cv2.morphologyEx(
        gray_roi,
        cv2.MORPH_CLOSE,
        background_kernel,
    )
    if timings is not None:
        timings["backgroundCloseMs"] = (
            time.perf_counter() - background_close_started
        ) * 1000.0
    ratio_percent = vision_profile["line_max_background_ratio_percent"]
    maximum_brightness = vision_profile["line_max_brightness"]
    binary_compare_started = (
        time.perf_counter() if timings is not None else 0.0
    )
    gray_16 = gray_roi.astype(np.uint16)
    background_16 = local_background.astype(np.uint16)
    np.multiply(gray_16, 100, out=gray_16)
    np.multiply(background_16, ratio_percent, out=background_16)
    relative_mask = cv2.compare(gray_16, background_16, cv2.CMP_LE)
    absolute_mask = cv2.inRange(gray_roi, 0, maximum_brightness)
    cv2.bitwise_and(relative_mask, absolute_mask, dst=relative_mask)
    binary_mask = relative_mask
    if timings is not None:
        timings["binaryCompareMs"] = (
            time.perf_counter() - binary_compare_started
        ) * 1000.0
    return binary_mask




def repair_small_specular_holes(binary_mask, color_roi, camera_format):
    """Preenche apenas reflexos claros, pequenos e internos à fita preta."""

    repair_status = {
        "specularRepairPixels": 0,
        "specularRepairComponents": 0,
    }
    if binary_mask.size == 0:
        return binary_mask, repair_status

    frame_height = color_roi.shape[0]
    maximum_diameter = max(1, int(round(
        SPECULAR_REPAIR_MAX_DIAMETER_PX * frame_height /
        SPECULAR_REPAIR_REFERENCE_FRAME_HEIGHT
    )))
    maximum_area = maximum_diameter * maximum_diameter
    try:
        hsv_roi = frame_to_hsv(color_roi, camera_format)
    except ValueError:
        # Sem a ordem de cores confirmada, não é seguro classificar um brilho.
        return binary_mask, repair_status

    contours, hierarchy = cv2.findContours(
        binary_mask.copy(),
        cv2.RETR_CCOMP,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if hierarchy is None:
        return binary_mask, repair_status

    repaired_mask = binary_mask.copy()
    for contour_index, contour in enumerate(contours):
        # No modo CCOMP, apenas os contornos com pai representam buracos
        # fechados. Regiões ligadas ao fundo sempre permanecem de fora.
        if hierarchy[0][contour_index][3] < 0:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if (width > maximum_diameter or height > maximum_diameter or
                cv2.contourArea(contour) > maximum_area):
            # Uma abertura larga pode ser uma separação real da fita.
            continue

        contour_in_roi = contour.copy()
        contour_in_roi[:, :, 0] -= x
        contour_in_roi[:, :, 1] -= y
        hole_shape = np.zeros((height, width), dtype=np.uint8)
        cv2.drawContours(hole_shape, [contour_in_roi], -1, 255, -1)
        hole_pixels = np.logical_and(
            hole_shape > 0,
            repaired_mask[y:y + height, x:x + width] == 0,
        )
        hole_pixel_count = int(np.count_nonzero(hole_pixels))
        if hole_pixel_count == 0 or hole_pixel_count > maximum_area:
            continue

        hole_hsv = hsv_roi[y:y + height, x:x + width]
        clear_and_unsaturated = np.logical_and(
            hole_hsv[:, :, 2] >= SPECULAR_REPAIR_MIN_VALUE,
            hole_hsv[:, :, 1] <= SPECULAR_REPAIR_MAX_SATURATION,
        )
        if not np.all(clear_and_unsaturated[hole_pixels]):
            # Verde e outras cores saturadas nunca podem virar preto.
            continue

        repaired_mask[y:y + height, x:x + width][hole_pixels] = 255
        repair_status["specularRepairPixels"] += hole_pixel_count
        repair_status["specularRepairComponents"] += 1
    return repaired_mask, repair_status


def create_filtered_line_mask(
    frame,
    vision_profile,
    camera_format="RGB888",
    return_repair_status=False,
    timings=None,
):
    """Segmenta a linha preta na parte inferior sem gerar decisões de controle."""

    frame_height = frame.shape[0]
    roi_start_y = int(round(frame_height * vision_profile["line_roi_start_ratio"]))
    line_roi = frame[roi_start_y:frame_height, :]
    gray_roi = cv2.cvtColor(line_roi, cv2.COLOR_BGR2GRAY)

    scaled_profile = dict(vision_profile)
    if "geometry_reference" in vision_profile:
        if "line_background_kernel_size" in scaled_profile:
            scaled_profile["line_background_kernel_size"] = scaled_odd_kernel_size(
                scaled_profile["line_background_kernel_size"],
                frame_height,
            )
        scaled_profile["open_kernel_size"] = scaled_odd_kernel_size(
            vision_profile["open_kernel_size"], frame_height
        )
        scaled_profile["close_kernel_size"] = scaled_odd_kernel_size(
            vision_profile["close_kernel_size"], frame_height
        )
    binary_started = time.perf_counter() if timings is not None else 0.0
    binary_mask = create_line_binary_mask(
        gray_roi,
        scaled_profile,
        timings=timings,
    )
    if timings is not None:
        timings["binaryMs"] = (
            time.perf_counter() - binary_started
        ) * 1000.0
    specular_started = time.perf_counter() if timings is not None else 0.0

    repair_status = {
        "specularRepairPixels": 0,
        "specularRepairComponents": 0,
    }
    if timings is not None:
        timings["specularMs"] = (
            time.perf_counter() - specular_started
        ) * 1000.0

    open_kernel_shape = (
        cv2.MORPH_ELLIPSE
        if scaled_profile.get("open_kernel_shape") == "ellipse"
        else cv2.MORPH_RECT
    )
    open_kernel = cached_structuring_element(
        open_kernel_shape,
        scaled_profile["open_kernel_size"],
        scaled_profile["open_kernel_size"],
    )
    close_kernel = cached_structuring_element(
        cv2.MORPH_RECT,
        scaled_profile["close_kernel_size"],
        scaled_profile["close_kernel_size"],
    )
    morph_open_started = time.perf_counter() if timings is not None else 0.0
    filtered_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, open_kernel)
    morph_open_ms = (
        (time.perf_counter() - morph_open_started) * 1000.0
        if timings is not None
        else 0.0
    )
    morph_close_started = time.perf_counter() if timings is not None else 0.0
    filtered_mask = cv2.morphologyEx(filtered_mask, cv2.MORPH_CLOSE, close_kernel)
    morph_close_ms = (
        (time.perf_counter() - morph_close_started) * 1000.0
        if timings is not None
        else 0.0
    )
    if timings is not None:
        timings["morphOpenMs"] = morph_open_ms
        timings["morphCloseMs"] = morph_close_ms
        timings["morphMs"] = morph_open_ms + morph_close_ms
    if return_repair_status:
        return filtered_mask, roi_start_y, repair_status
    return filtered_mask, roi_start_y


def scale_reference_y(reference_y, reference_height, frame_height):
    """Converte uma coordenada vertical de referência para a altura real."""

    return int(round(frame_height * reference_y / reference_height))


def resolve_vision_geometry(frame_height, vision_profile):
    """Converte os limites verticais da máscara preta e da visão verde."""

    geometry_reference = vision_profile.get("geometry_reference")
    if geometry_reference is None:
        return {
            "structural_end_y": frame_height,
            "green_end_y": frame_height,
            "ignored_start_y": None,
            "pixel_scale": 1.0,
        }

    reference_height = geometry_reference["frame_height"]
    structural_end_y = scale_reference_y(
        geometry_reference["structural_end_y"],
        reference_height,
        frame_height,
    )
    green_end_y = scale_reference_y(
        geometry_reference.get(
            "green_end_y",
            geometry_reference["structural_end_y"],
        ),
        reference_height,
        frame_height,
    )
    if not 0 < structural_end_y <= frame_height:
        raise ValueError("O limite estrutural da câmera está fora do frame.")
    if not 0 < green_end_y <= frame_height:
        raise ValueError("O limite da visão verde está fora do frame.")

    return {
        "structural_end_y": structural_end_y,
        "green_end_y": green_end_y,
        "ignored_start_y": structural_end_y,
        "pixel_scale": float(frame_height) / float(reference_height),
    }

def create_structural_line_mask(
    filtered_mask,
    roi_start_y,
    structural_end_y,
    structural_start_y=0,
):
    """Remove da máscara as áreas físicas que não podem gerar candidatos."""

    structural_start_in_roi = max(
        0,
        min(filtered_mask.shape[0], structural_start_y - roi_start_y),
    )

    structural_end_in_roi = max(
        0,
        min(filtered_mask.shape[0], structural_end_y - roi_start_y),
    )

    if (
        structural_start_in_roi <= 0
        and structural_end_in_roi >= filtered_mask.shape[0]
    ):
        return filtered_mask

    structural_mask = filtered_mask.copy()
    structural_mask[:structural_start_in_roi, :] = 0
    structural_mask[structural_end_in_roi:, :] = 0

    return structural_mask


def component_has_min_thickness(component_mask):
    """
    Rejeita componentes predominantemente finos,
    como frestas entre placas da pista.

    Não basta existir um único ponto grosso:
    uma fração relevante do componente precisa
    possuir espessura compatível com a fita.
    """

    if component_mask.size == 0:
        return False

    component_pixels = cv2.countNonZero(
        component_mask
    )

    if component_pixels == 0:
        return False

    distance_map = cv2.distanceTransform(
        component_mask,
        cv2.DIST_L2,
        3,
    )

    minimum_radius = (
        LINE_MIN_COMPONENT_THICKNESS_PX
        / 2.0
    )

    thick_core_pixels = int(
        np.count_nonzero(
            distance_map >= minimum_radius
        )
    )

    thick_core_ratio = (
        float(thick_core_pixels)
        / float(component_pixels)
    )

    return (
        thick_core_ratio
        >= LINE_MIN_COMPONENT_CORE_RATIO
    )


def create_line_candidate_mask(
    structural_mask,
    vision_profile,
):
    """Mantém somente componentes compatíveis com a fita preta."""

    full_contours, _ = cv2.findContours(
        structural_mask.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    accepted_contours = []

    minimum_short_side_px = (
        min(structural_mask.shape[:2])
        * vision_profile[
            "full_line_min_short_side_ratio"
        ]
    )

    maximum_area = (
        structural_mask.size
        * vision_profile.get(
            "full_line_max_area_ratio",
            1.0,
        )
    )

    for contour in full_contours:
        contour_area = cv2.contourArea(
            contour
        )

        # Componentes minúsculos não podem representar
        # uma faixa útil para o robô.
        if (
            contour_area
            < LINE_MIN_COMPONENT_AREA_PX
        ):
            continue

        if contour_area > maximum_area:
            continue

        rect = cv2.minAreaRect(
            contour
        )

        width, height = rect[1]

        if (
            not math.isfinite(width)
            or not math.isfinite(height)
            or width <= 0.0
            or height <= 0.0
        ):
            continue

        short_side_px = min(
            width,
            height,
        )

        if (
            short_side_px
            < minimum_short_side_px
        ):
            continue

        # Analisa a espessura real do próprio componente.
        # Isso evita aceitar uma fresta longa apenas porque
        # sua caixa rotacionada ficou larga.
        component_mask = np.zeros_like(
            structural_mask
        )

        cv2.drawContours(
            component_mask,
            [contour],
            -1,
            255,
            cv2.FILLED,
        )

        if not component_has_min_thickness(
            component_mask
        ):
            continue

        accepted_contours.append(
            contour
        )

    line_candidate_mask = (
        structural_mask.copy()
    )

    line_candidate_mask.fill(0)

    if accepted_contours:
        cv2.drawContours(
            line_candidate_mask,
            accepted_contours,
            -1,
            255,
            cv2.FILLED,
        )

    return line_candidate_mask



def draw_line_control_overlay(frame, line_follower_command):
    """Desenha o estado principal e as potências no topo da câmera inferior."""

    processing_ms = finite_virtual_position(
        line_follower_command.get("lineProcessingMs")
    )
    line_state = str(
        line_follower_command.get("lineState", "INVALID")
    ).strip().upper() or "INVALID"
    virtual_state = str(
        line_follower_command.get("virtualState", "INVALID")
    ).strip().upper() or "INVALID"
    display_state = line_state
    if (
        line_state == "LINE"
        and virtual_state in (
            VIRTUAL_STATE_REORIENT_LEFT,
            VIRTUAL_STATE_REORIENT_RIGHT,
        )
    ):
        display_state = "REORIENT"

    processing_text = (
        f"{display_state} {processing_ms:.1f}ms"
        if processing_ms is not None
        else f"{display_state} --ms"
    )

    overlay_texts = (
        processing_text,
        (
            f"L {float(line_follower_command['left_power']):.2f}  "
            f"R {float(line_follower_command['right_power']):.2f}"
        ),
    )
    for line_index, overlay_text in enumerate(overlay_texts):
        cv2.putText(
            frame,
            overlay_text,
            (8, 22 + line_index * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )




def encode_frame(frame):
    """Converte para JPEG o frame com a visualização experimental."""

    parameters = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
    ok, encoded = cv2.imencode(".jpg", frame, parameters)
    return encoded.tobytes() if ok else None


def publish_stream_frame(jpeg):
    global latest_jpeg, latest_jpeg_sequence
    with frame_condition:
        latest_jpeg = jpeg
        latest_jpeg_sequence += 1
        frame_condition.notify_all()


def save_frame(jpeg):
    with open(TEMP_FRAME_PATH, "wb") as frame_file:
        frame_file.write(jpeg)
    os.replace(TEMP_FRAME_PATH, FRAME_PATH)


def save_line_status(
    line_follower_command,
    line_timestamp,
    line_sequence,
    green_status,
    specular_repair_status=None,
):
    """Publica controle visual e telemetria leve no IPC rápido da linha."""

    try:
        line_timestamp = float(line_timestamp)
        if not math.isfinite(line_timestamp):
            raise ValueError("lineTimestamp inválido")
        if (
            not isinstance(line_sequence, int)
            or isinstance(line_sequence, bool)
            or line_sequence < 0
        ):
            raise ValueError("lineSequence inválido")

        normal_left = float(line_follower_command["left_power"])
        normal_right = float(line_follower_command["right_power"])
        if not all(
            math.isfinite(value) and -1.0 <= value <= 1.0
            for value in (normal_left, normal_right)
        ):
            raise ValueError("Comando visual fora da faixa normalizada")

        repair_status = specular_repair_status or {}
        far_trusted = virtual_sensor_trust_is_active(
            line_follower_command,
            "farTrusted",
        )
        medium_trusted = virtual_sensor_trust_is_active(
            line_follower_command,
            "mediumTrusted",
        )
        line_status = {
            "lineFollowerLeftPower": normal_left,
            "lineFollowerRightPower": normal_right,
            "lineNearDetected": virtual_sensor_is_active(
                line_follower_command["nearCenter"]
            ),
            "lineControlSource": str(
                line_follower_command.get("controlSource", "unknown")
            ),
            "nearFinePosition": finite_virtual_position(
                line_follower_command["nearFinePosition"]
            ),
            "farLineConfidence": normalized_line_confidence(
                line_follower_command.get("farLineConfidence")
            ),
            "farThicknessConsistency": normalized_line_confidence(
                line_follower_command.get("farThicknessConsistency")
            ),
            "farTrusted": far_trusted,
            "mediumPosition": (
                finite_virtual_position(
                    line_follower_command.get("mediumPosition")
                )
                if medium_trusted
                else None
            ),
            "mediumLineConfidence": normalized_line_confidence(
                line_follower_command.get("mediumLineConfidence")
            ),
            "mediumThicknessConsistency": normalized_line_confidence(
                line_follower_command.get("mediumThicknessConsistency")
            ),
            "mediumTrusted": medium_trusted,
            "farBandPosition": (
                finite_virtual_position(
                    line_follower_command.get("farBandPosition")
                )
                if far_trusted
                else None
            ),
            "headingAngleDeg": finite_virtual_position(
                line_follower_command.get("headingAngle")
            ),
            "finalSteering": finite_virtual_position(
                line_follower_command.get("finalSteering")
            ),
            "vstate": str(
                line_follower_command.get("virtualState", "INVALID")
            ),
            "lineState": str(
                line_follower_command.get("lineState", "INVALID")
            ),
            "trustedDirection": str(
                line_follower_command.get("trustedDirection", "NONE")
            ),
            "lineTimestamp": line_timestamp,
            "lineSequence": line_sequence,
            "specularRepairPixels": max(
                0, int(repair_status.get("specularRepairPixels", 0))
            ),
            "specularRepairComponents": max(
                0, int(repair_status.get("specularRepairComponents", 0))
            ),
        }
        line_status.update(green_status)
        with open(TEMP_LINE_STATUS_PATH, "w", encoding="utf-8") as status_file:
            json.dump(line_status, status_file, allow_nan=False)
        os.replace(TEMP_LINE_STATUS_PATH, LINE_STATUS_PATH)
    except (KeyError, OSError, TypeError, ValueError) as error:
        print(
            f"Falha ao publicar telemetria rápida da linha: {error}",
            flush=True,
        )


def save_status(
    fps,
    camera_profile,
    camera_details,
    camera_format="",
    active=True,
    error_message="",
    line_timestamp=0.0,
    line_sequence=0,
    specular_repair_status=None,
    green_status=None,
    line_timings=None,
    far_line_confidence=0.0,
    medium_line_confidence=0.0,
    far_thickness_consistency=0.0,
    medium_thickness_consistency=0.0,
):
    """Publica somente a saúde da câmera e os resultados visuais preservados."""

    try:
        line_timestamp = float(line_timestamp)
        line_timestamp_valid = (
            math.isfinite(line_timestamp) and line_timestamp >= 0.0
        )
    except (TypeError, ValueError):
        line_timestamp_valid = False
        line_timestamp = 0.0

    line_sequence_valid = (
        isinstance(line_sequence, int)
        and not isinstance(line_sequence, bool)
        and line_sequence >= 0
    )
    if not line_sequence_valid:
        line_sequence = 0

    frame_width, frame_height = camera_profile["main_size"]
    line_ipc_fresh = (
        camera_profile["role"] != "down"
        or (
            line_timestamp_valid
            and line_timestamp > 0.0
            and time.time() - line_timestamp <= 0.5
        )
    )
    repair_status = specular_repair_status or {}
    repaired_pixels = max(
        0, int(repair_status.get("specularRepairPixels", 0))
    )
    repaired_components = max(
        0, int(repair_status.get("specularRepairComponents", 0))
    )
    timing_status = line_timings if isinstance(line_timings, dict) else {}
    safe_line_timings = {}
    for timing_name in (
        "lineProcessingMs",
        "binaryMs",
        "backgroundKernelMs",
        "backgroundCloseMs",
        "binaryCompareMs",
        "specularMs",
        "morphMs",
        "morphOpenMs",
        "morphCloseMs",
        "contoursMs",
    ):
        try:
            timing_value = float(timing_status.get(timing_name, 0.0))
        except (TypeError, ValueError):
            timing_value = 0.0
        if not math.isfinite(timing_value) or timing_value < 0.0:
            timing_value = 0.0
        safe_line_timings[timing_name] = timing_value
    status = {
        "fps": round(fps, 2),
        "active": active,
        # O processo inferior só existe enquanto seu gerenciador está ativo.
        "enabled": camera_profile["role"] == "down",
        "state": "ONLINE" if active and line_ipc_fresh else (
            "FALHA" if error_message else "INICIANDO"
        ),
        "lineIpcFresh": line_ipc_fresh,
        "cameraRole": camera_profile["role"],
        "width": frame_width,
        "height": frame_height,
        "mainResolution": {
            "width": frame_width,
            "height": frame_height,
        },
        "jpegQuality": JPEG_QUALITY,
        "targetCameraFps": camera_profile["target_fps"],
        "cameraFormat": camera_format,
        "rotationDegrees": camera_details.get(
            "rotationDegrees",
            camera_profile["rotation_degrees"],
        ),
        "cameraIndex": camera_details.get("cameraIndex"),
        "cameraId": camera_details.get("cameraId", ""),
        "cameraModel": camera_details.get("cameraModel", ""),
        "sensorMode": camera_details.get("sensorMode"),
        "scalerCrop": camera_details.get("scalerCrop"),
        "transform": camera_details.get(
            "transform",
            camera_transform_settings(camera_profile)["name"],
        ),
        "streamPort": MJPEG_STREAM_PORT,
        "streamPath": MJPEG_STREAM_PATH,
        "streamFps": MJPEG_STREAM_FPS,
        "error": error_message,
        "timestamp": time.time(),
        "lineFollowerImplemented": False,
        "lineTimestamp": line_timestamp,
        "lineSequence": line_sequence,
        "farLineConfidence": normalized_line_confidence(
            far_line_confidence
        ),
        "mediumLineConfidence": normalized_line_confidence(
            medium_line_confidence
        ),
        "farThicknessConsistency": normalized_line_confidence(
            far_thickness_consistency
        ),
        "mediumThicknessConsistency": normalized_line_confidence(
            medium_thickness_consistency
        ),
        "specularRepairPixels": repaired_pixels,
        "specularRepairComponents": repaired_components,
        "lineProcessingMs": safe_line_timings["lineProcessingMs"],
        "binaryMs": safe_line_timings["binaryMs"],
        "backgroundKernelMs": safe_line_timings["backgroundKernelMs"],
        "backgroundCloseMs": safe_line_timings["backgroundCloseMs"],
        "binaryCompareMs": safe_line_timings["binaryCompareMs"],
        "specularMs": safe_line_timings["specularMs"],
        "morphMs": safe_line_timings["morphMs"],
        "morphOpenMs": safe_line_timings["morphOpenMs"],
        "morphCloseMs": safe_line_timings["morphCloseMs"],
        "contoursMs": safe_line_timings["contoursMs"],
    }
    status.update(green_status or empty_green_status())
    with open(TEMP_STATUS_PATH, "w", encoding="utf-8") as status_file:
        json.dump(status, status_file, allow_nan=False)
    os.replace(TEMP_STATUS_PATH, STATUS_PATH)


def main():
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    camera_profile = parse_camera_profile()
    vision_profile = camera_profile["vision"]
    camera_details = {}

    if GPIO is None or Picamera2 is None or Transform is None:
        error_message = "Dependências GPIO, libcamera ou Picamera2 não encontradas."
        print(error_message, flush=True)
        save_status(
            0.0,
            camera_profile,
            camera_details,
            active=False,
            error_message=error_message,
        )
        return 1

    save_status(0.0, camera_profile, camera_details)
    stream_server = None
    picam2 = None
    camera_started = False
    light_ready = False

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

        previous_time = time.monotonic()
        last_stream_time = 0.0
        last_snapshot_time = 0.0
        last_status_time = 0.0
        smoothed_fps = 0.0
        line_sequence = 0
        green_tracker = GreenObservationTracker()
        virtual_turn_tracker = VirtualTurnStateTracker()
        pivot_state_tracker = VirtualPivotStateTracker()
        medium_spin_tracker = VirtualMediumSpinTracker()
        line_search_tracker = VirtualLineSearchTracker()

        # Estado persistente das manobras sinalizadas por verde.
        direcao_verde_ativa = "NENHUMA"
        curva_verde_iniciada = False
        quadros_centralizado_verde = 0
        quadros_verde_ativo = 0

         # Estado persistente da travessia de gap.
        gap_forward_active = False
        gap_forward_frames = 0
        gap_reacquire_frames = 0
        gap_line_lost_seen = False
        gap_recent_near_frames = 0

        # Impede que o mesmo marcador verde seja aceito novamente.
        verde_armado = True
        quadros_sem_verde = 0

        green_processing_enabled = bool(
            vision_profile.get("green_detection_enabled", False)
            and GREEN_PROCESSING_ENABLED
        )
        print(
            "Visão verde para overlay e telemetria: "
            f"{'ligada' if green_processing_enabled else 'desligada'}.",
            flush=True,
        )

        while running:
            green_capture_requested = os.path.isfile(GREEN_CAPTURE_REQUEST_PATH)
            green_capture_metadata = {}
            if green_capture_requested:
                # A captura preserva metadados do mesmo frame usado no diagnóstico.
                camera_request = picam2.capture_request()
                try:
                    raw_frame = camera_request.make_array("main")
                    green_capture_metadata = camera_request.get_metadata()
                finally:
                    camera_request.release()
            else:
                raw_frame = picam2.capture_array()

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
            line_candidate_mask = create_line_candidate_mask(
                structural_mask,
                vision_profile,
            )
            line_timings["contoursMs"] = (
                time.perf_counter() - contours_started
            ) * 1000.0
            line_vision_ms = (
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
            green_status["greenDecisionState"] = (
                "accepted"
                if green_status["greenConfirmed"]
                else "candidate"
                if green_raw_interpretation != "SEM_DECISAO"
                else "idle"
            )

                        # Um verde confirmado é aceito apenas quando o sistema está armado
            # e nenhuma outra direção verde está sendo executada.
            if (
                verde_armado
                and direcao_verde_ativa == "NENHUMA"
                and green_status["greenConfirmed"]
            ):
                interpretacao_verde = green_status["greenInterpretation"]

                if interpretacao_verde == "ESQUERDA":
                    direcao_verde_ativa = "ESQUERDA"
                    curva_verde_iniciada = False
                    quadros_centralizado_verde = 0
                    quadros_verde_ativo = 0
                    line_search_tracker.stop()
                    verde_armado = False
                    quadros_sem_verde = 0

                elif interpretacao_verde == "DIREITA":
                    direcao_verde_ativa = "DIREITA"
                    curva_verde_iniciada = False
                    quadros_centralizado_verde = 0
                    quadros_verde_ativo = 0
                    line_search_tracker.stop()
                    verde_armado = False
                    quadros_sem_verde = 0

            # Depois que um verde foi aceito, o sistema só poderá ser armado
            # novamente após vários quadros consecutivos sem nenhum candidato verde. % isaque hulk verde
            if not verde_armado:
                if green_status["greenCandidateCount"] == 0:
                    quadros_sem_verde += 1
                else:
                    quadros_sem_verde = 0

            virtual_sensors = read_virtual_line_sensors(
                line_candidate_mask,
                direcao_verde_ativa,
            )
            near_center_visible = virtual_sensor_is_active(
                virtual_sensors["nearCenter"]
            )
            gap_recent_near_frames = update_gap_recent_near_frames(
                gap_recent_near_frames,
                near_center_visible,
            )
            raw_line_visible = virtual_raw_line_is_visible(
                virtual_sensors
            )
            green_timeout_state = update_green_maneuver_state(
                direcao_verde_ativa,
                quadros_verde_ativo,
                raw_line_visible,
            )
            sensor_recovery_requested = False
            if green_timeout_state["timedOut"]:
                direcao_verde_ativa = green_timeout_state["direction"]
                quadros_verde_ativo = green_timeout_state["activeFrames"]
                curva_verde_iniciada = False
                quadros_centralizado_verde = 0
                search_direction = green_timeout_state["searchDirection"]
                if search_direction is not None:
                    line_search_tracker.start(search_direction)
                else:
                    sensor_recovery_requested = True
                # Remove a máscara verde já no mesmo frame do timeout.
                virtual_sensors = read_virtual_line_sensors(
                    line_candidate_mask,
                    direcao_verde_ativa,
                )
            else:
                direcao_verde_ativa = green_timeout_state["direction"]
                quadros_verde_ativo = green_timeout_state["activeFrames"]

            geometric_guidance = extract_gap_geometric_guidance(
                line_candidate_mask,
                gap_forward_active,
                direcao_verde_ativa,
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
            geometric_heading = geometric_guidance.get(
                "farHeadingDeg"
            )
            lateral_exit_target = geometric_guidance.get(
                "lateralExitTarget"
            )
            real_near_point = geometric_guidance.get(
                "nearPoint"
            )
            virtual_near_point = geometric_guidance.get(
                "virtualNearPoint"
            )
            trace_folded_back = (
                geometric_heading is not None
                and abs(float(geometric_heading)) > 90.0
            )

            if gap_entry_is_required(
                gap_forward_active,
                direcao_verde_ativa,
                gap_recent_near_frames,
                near_center_visible,
                real_near_point,
                virtual_near_point,
                lateral_exit_target,
            ):
                gap_forward_active = True
                gap_forward_frames = 0
                gap_reacquire_frames = 0
                gap_line_lost_seen = True
                

            gap_blind_search_requested = False
            if gap_forward_active:
                near_center_reacquired = virtual_sensor_is_active(
                    virtual_sensors["nearCenter"]
                )
                near_reacquired = (
                    near_center_reacquired
                    or (
                        real_near_point is not None
                        and not trace_folded_back
                    )
                )
                gap_state = update_gap_forward_recovery(
                    gap_forward_active,
                    gap_forward_frames,
                    gap_reacquire_frames,
                    gap_line_lost_seen,
                    near_reacquired,
                )
                gap_forward_active = gap_state["active"]
                gap_forward_frames = gap_state["forwardFrames"]
                gap_reacquire_frames = gap_state["reacquireFrames"]
                gap_line_lost_seen = gap_state["lineLostSeen"]
                gap_blind_search_requested = gap_state[
                    "blindSearchRequested"
                ]
                if not gap_forward_active:
                    line_search_tracker.stop()

            line_control_started = time.perf_counter()

            line_follower_command = (
                calculate_line_follower_command(
                    line_candidate_mask,
                    green_status,
                    direcao_verde_ativa,
                    gap_forward_active,
                    virtual_turn_tracker=virtual_turn_tracker,
                    pivot_state_tracker=pivot_state_tracker,
                    medium_spin_tracker=medium_spin_tracker,
                    virtual_sensors=virtual_sensors,
                    line_search_tracker=line_search_tracker,
                    blind_search_requested=gap_blind_search_requested,
                    sensor_recovery_requested=sensor_recovery_requested,
                )
            )

            near_fine_position = line_follower_command["nearFinePosition"]

            # Confirma que o robô realmente começou a entrar no ramo
            # indicado pelo marcador verde.
            if not curva_verde_iniciada:
                if (
                    direcao_verde_ativa == "ESQUERDA"
                    and near_fine_position is not None
                    and near_fine_position <= -LIMIAR_CURVA_VERDE_INICIADA
                ):
                    curva_verde_iniciada = True
                    quadros_centralizado_verde = 0

                elif (
                    direcao_verde_ativa == "DIREITA"
                    and near_fine_position is not None
                    and near_fine_position >= LIMIAR_CURVA_VERDE_INICIADA
                ):
                    curva_verde_iniciada = True
                    quadros_centralizado_verde = 0

            # Depois que a curva começou, espera a posição fina local voltar
            # ao centro por vários quadros consecutivos. Isso indica que o
            # robô já entrou e se alinhou com a nova faixa.
            if (
                direcao_verde_ativa != "NENHUMA"
                and curva_verde_iniciada
            ):
                if (
                    near_fine_position is not None
                    and abs(near_fine_position)
                    <= LIMIAR_CENTRALIZACAO_VERDE
                ):
                    quadros_centralizado_verde += 1
                else:
                    quadros_centralizado_verde = 0

                if (
                    quadros_centralizado_verde
                    >= QUADROS_CENTRALIZADO_PARA_CONCLUIR
                ):
                    completed_green_state = update_green_maneuver_state(
                        direcao_verde_ativa,
                        quadros_verde_ativo,
                        raw_line_visible,
                        completed=True,
                    )
                    direcao_verde_ativa = completed_green_state["direction"]
                    quadros_verde_ativo = completed_green_state[
                        "activeFrames"
                    ]
                    curva_verde_iniciada = False
                    quadros_centralizado_verde = 0
                    line_search_tracker.stop()

            # A curva já pode ter terminado, mas um novo verde só será
            # aceito depois de X quadros consecutivos sem candidato verde.
            if (
                not verde_armado
                and direcao_verde_ativa == "NENHUMA"
                and quadros_sem_verde >= QUADROS_PARA_REARMAR_VERDE
            ):
                verde_armado = True
                quadros_sem_verde = 0

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
                save_line_status(
                    line_follower_command,
                    line_timestamp,
                    line_sequence,
                    green_status,
                    specular_repair_status=specular_repair_status,
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
                        CAMERA_EXPOSURE_VALUE,
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

            display_mode = get_display_mode()
            frame = create_display_frame(
                raw_frame,
                line_candidate_mask,
                green_mask,
                roi_start_y,
                display_mode,
                structural_mask,
            )

            if camera_profile["role"] == "down":
                draw_line_control_overlay(frame, line_follower_command)
                draw_virtual_sensor_geometry(
                    frame,
                    line_follower_command,
                    show_debug_details=(
                        display_mode in (
                            DISPLAY_MODE_LINE,
                            DISPLAY_MODE_REAL,
                        )
                    ),
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
                save_status(
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
                )
                last_status_time = now
    except Exception as error:
        import traceback
        traceback.print_exc()
        error_message = f"Camera script failed: {error}"
        print(error_message, flush=True)
        save_status(
            0.0,
            camera_profile,
            camera_details,
            active=False,
            error_message=error_message,
        )
        return 1
    finally:
        if stream_server is not None:
            stream_server.shutdown()
            stream_server.server_close()
        if GPIO is not None and light_ready:
            GPIO.output(LIGHT_PIN_BOARD, GPIO.LOW)
            GPIO.cleanup()
        if picam2 is not None and camera_started:
            picam2.stop()

    return 0
if __name__ == "__main__":
    raise SystemExit(main())
