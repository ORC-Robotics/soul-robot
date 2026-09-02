"""Publicação atômica da telemetria rápida do seguidor de linha."""

import json
import math
import os

from .numeric import finite_virtual_position
from .reorient import virtual_sensor_trust_is_active
from .virtual_sensors import (
    normalized_line_confidence,
    virtual_sensor_is_active,
)


LINE_STATUS_PATH = "/dev/shm/obr_line_status.json"
TEMP_LINE_STATUS_PATH = "/dev/shm/obr_line_status.tmp.json"


def save_line_status(
    line_follower_command,
    line_timestamp,
    line_sequence,
    green_status,
    specular_repair_status=None,
    status_path=None,
    temp_status_path=None,
):
    """Publica controle visual e telemetria leve no IPC rápido da linha."""

    status_path = LINE_STATUS_PATH if status_path is None else status_path
    temp_status_path = (
        TEMP_LINE_STATUS_PATH
        if temp_status_path is None
        else temp_status_path
    )

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
        with open(temp_status_path, "w", encoding="utf-8") as status_file:
            json.dump(line_status, status_file, allow_nan=False)
        os.replace(temp_status_path, status_path)
    except (KeyError, OSError, TypeError, ValueError) as error:
        print(
            f"Falha ao publicar telemetria rápida da linha: {error}",
            flush=True,
        )


class LineStatusPublisher:
    """Mantém os destinos do IPC fora do processamento da câmera."""

    def __init__(self, status_path=None, temp_status_path=None):
        self.status_path = status_path
        self.temp_status_path = temp_status_path

    def publish(
        self,
        line_follower_command,
        line_timestamp,
        line_sequence,
        green_status,
        specular_repair_status=None,
    ):
        """Publica um frame usando substituição atômica do arquivo JSON."""

        return save_line_status(
            line_follower_command,
            line_timestamp,
            line_sequence,
            green_status,
            specular_repair_status=specular_repair_status,
            status_path=self.status_path,
            temp_status_path=self.temp_status_path,
        )
