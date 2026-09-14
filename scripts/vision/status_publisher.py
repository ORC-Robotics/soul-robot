"""Publicação atômica dos status da câmera e da linha."""

import json
import math
import os
import time
try:
    from silver_dataset_recorder import read_dataset_status
except ImportError:
    # A ausência da coleta opcional não impede a publicação da visão.
    def read_dataset_status():
        return {}
from .camera import (
    camera_transform_settings,
)
from .camera_config import (
    JPEG_QUALITY,
    LEGACY_LINE_DEBUG_ENABLED,
    LINE_STATUS_PATH,
    MJPEG_STREAM_FPS,
    MJPEG_STREAM_PATH,
    MJPEG_STREAM_PORT,
    STATUS_PATH,
    TEMP_LINE_STATUS_PATH,
    TEMP_STATUS_PATH,
)
from .fusion_guidance import (
    empty_fusion_style_line,
)
from .green_detection import (
    empty_green_status,
)
from .normal_trajectory import (
    empty_normal_trajectory,
)
from .silver_detection import (
    empty_silver_shadow_status,
)
from .numeric import (
    finite_virtual_position,
)
from .virtual_sensors import (
    normalized_line_confidence,
    virtual_sensor_is_active,
    virtual_sensor_trust_is_active,
)

def save_line_status(
    line_follower_command,
    line_timestamp,
    line_sequence,
    green_status,
    specular_repair_status=None,
    silver_status=None,
    red_status=None,
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
            "fusionAngle": finite_virtual_position(
                line_follower_command.get("fusionAngle")
            ),
            "filteredFusionAngle": finite_virtual_position(
                line_follower_command.get("filteredFusionAngle")
            ),
            "fusionSteeringError": finite_virtual_position(
                line_follower_command.get("fusionSteeringError")
            ),
            "fusionControlActive": (
                line_follower_command.get("fusionControlActive") is True
            ),
            "fusionPreferredDirection": str(
                line_follower_command.get(
                    "fusionPreferredDirection",
                    "NONE",
                )
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
            "illuminationCorrectionActive": (
                repair_status.get("illuminationCorrectionActive") is True
            ),
            "illuminationCorrectionMs": max(
                0.0,
                float(repair_status.get("illuminationCorrectionMs", 0.0)),
            ),
            "illuminationReferenceSha256": str(
                repair_status.get("illuminationReferenceSha256", "")
            ),
            "illuminationDarkPixelsPreserved": max(
                0,
                int(repair_status.get("illuminationDarkPixelsPreserved", 0)),
            ),
        }
        line_status.update(green_status)
        # Referência e decisão são diagnóstico/validação; não substituem potências.
        for key in ("bottomPathReference", "gapValidationDecision", "gapValidationReason",
                    "nearLinePresent", "nearLineState", "nearLineMissingFrames",
                    "nearLineActivePixels",
                    "fusionLateralCurveContinuation",
                    "bottomFarLinePresent", "bottomFarPresentFrames",
                    "bottomFusionReacquireCandidate", "bottomFusionPathConnected",
                    "bottomFusionReacquireFrames",
                    "bottomFusionReacquireReady", "forwardPresenceState"):
            if key in line_follower_command:
                line_status[key] = line_follower_command[key]
        if isinstance(red_status, dict):
            line_status.update(red_status)
        marker_status = silver_status if isinstance(silver_status, dict) else {}
        line_status["exitLineUnbranched"] = line_follower_command.get("exitLineUnbranched") is True
        line_status["silverClassifierAvailable"] = marker_status.get("silverShadowAvailable") is True
        line_status["silverTimestamp"] = marker_status.get("silverShadowTimestamp", 0.0)
        line_status["silverSequence"] = marker_status.get("silverShadowSequence", 0)
        line_status["courseMarkerConfirmed"] = (
            marker_status.get("courseMarkerConfirmed") is True
        )
        # A candidata preliminar autoriza somente o avanço curto por encoder.
        line_status["silverCandidateDetected"] = (
            marker_status.get("silverShadowDetected") is True
        )
        line_status["courseMarker"] = (
            "GRAY"
            if line_status["courseMarkerConfirmed"]
            and marker_status.get("courseMarker") == "GRAY"
            else "NONE"
        )
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
    normal_trajectory=None,
    fusion_style_line=None,
    line_follower_command=None,
    silver_shadow_status=None,
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
    control_status = (
        line_follower_command
        if isinstance(line_follower_command, dict)
        else {}
    )
    safe_line_timings = {}
    for timing_name in (
        "lineProcessingMs",
        "illuminationCorrectionMs",
        "binaryMs",
        "backgroundKernelMs",
        "backgroundCloseMs",
        "binaryCompareMs",
        "specularMs",
        "morphMs",
        "morphOpenMs",
        "morphCloseMs",
        "contoursMs",
        "normalTrajectoryMs",
        "fusionStyleMs",
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
        "illuminationCorrectionConfigured": (
            repair_status.get("illuminationCorrectionConfigured") is True
        ),
        "illuminationCorrectionActive": (
            repair_status.get("illuminationCorrectionActive") is True
        ),
        "illuminationCorrectionError": str(
            repair_status.get("illuminationCorrectionError", "")
        ),
        "illuminationReferenceSha256": str(
            repair_status.get("illuminationReferenceSha256", "")
        ),
        "illuminationMaximumGain": finite_virtual_position(
            repair_status.get("illuminationMaximumGain")
        ),
        "illuminationTargetGray": finite_virtual_position(
            repair_status.get("illuminationTargetGray")
        ),
        "illuminationCorrectionZonePercent": finite_virtual_position(
            repair_status.get("illuminationCorrectionZonePercent")
        ),
        "illuminationCorrectionMs": safe_line_timings[
            "illuminationCorrectionMs"
        ],
        "illuminationDarkPixelsPreserved": max(
            0,
            int(repair_status.get("illuminationDarkPixelsPreserved", 0)),
        ),
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
        "normalTrajectoryMs": safe_line_timings["normalTrajectoryMs"],
        "fusionStyleMs": safe_line_timings["fusionStyleMs"],
        "normalTrajectory": (
            normal_trajectory
            if isinstance(normal_trajectory, dict)
            else empty_normal_trajectory()
        ),
        "fusionStyleLine": (
            fusion_style_line
            if isinstance(fusion_style_line, dict)
            else empty_fusion_style_line()
        ),
        "legacyLineDebugEnabled": bool(LEGACY_LINE_DEBUG_ENABLED),
        "fusionAngle": finite_virtual_position(
            control_status.get("fusionAngle")
        ),
        "filteredFusionAngle": finite_virtual_position(
            control_status.get("filteredFusionAngle")
        ),
        "fusionSteeringError": finite_virtual_position(
            control_status.get("fusionSteeringError")
        ),
        "fusionControlActive": (
            control_status.get("fusionControlActive") is True
        ),
        "fusionPreferredDirection": str(
            control_status.get("fusionPreferredDirection", "NONE")
        ),
        "lineControlSource": str(
            control_status.get("controlSource", "unknown")
        ),
    }
    status.update(green_status or empty_green_status())
    status.update(
        silver_shadow_status
        if isinstance(silver_shadow_status, dict)
        else empty_silver_shadow_status()
    )
    status.update(read_dataset_status())
    with open(TEMP_STATUS_PATH, "w", encoding="utf-8") as status_file:
        json.dump(status, status_file, allow_nan=False)
    os.replace(TEMP_STATUS_PATH, STATUS_PATH)


class LineStatusPublisher:
    """Possui os caminhos usados para publicar status por troca atômica."""

    def __init__(
        self,
        line_status_path=LINE_STATUS_PATH,
        temp_line_status_path=TEMP_LINE_STATUS_PATH,
        status_path=STATUS_PATH,
        temp_status_path=TEMP_STATUS_PATH,
    ):
        self.line_status_path = line_status_path
        self.temp_line_status_path = temp_line_status_path
        self.status_path = status_path
        self.temp_status_path = temp_status_path

    def save_line_status(self, *args, **kwargs):
        """Publica o status rápido nos caminhos pertencentes a esta instância."""

        global LINE_STATUS_PATH, TEMP_LINE_STATUS_PATH
        previous_paths = LINE_STATUS_PATH, TEMP_LINE_STATUS_PATH
        LINE_STATUS_PATH = self.line_status_path
        TEMP_LINE_STATUS_PATH = self.temp_line_status_path
        try:
            return save_line_status(*args, **kwargs)
        finally:
            LINE_STATUS_PATH, TEMP_LINE_STATUS_PATH = previous_paths

    def save_status(self, *args, **kwargs):
        """Publica o status completo nos caminhos pertencentes a esta instância."""

        global STATUS_PATH, TEMP_STATUS_PATH
        previous_paths = STATUS_PATH, TEMP_STATUS_PATH
        STATUS_PATH = self.status_path
        TEMP_STATUS_PATH = self.temp_status_path
        try:
            return save_status(*args, **kwargs)
        finally:
            STATUS_PATH, TEMP_STATUS_PATH = previous_paths
