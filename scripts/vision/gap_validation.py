"""Confirma GAP com evidência frontal e devolve perdas ao recovery inferior."""

import json
import math

import cv2
import numpy as np

from .camera_config import (GAP_VALIDATION_CONFIG, NEAR_VIRTUAL_SENSOR_CONFIG,
                            VIRTUAL_BLIND_SEARCH_BACKUP_FRAMES, GEOMETRIC_GAP_REACQUIRE_FRAMES)
from .forward_path import bottom_reference
from .line_control import (gap_entry_is_required, update_gap_forward_recovery,
                           virtual_far_line_is_visible)
from .virtual_sensors import (read_virtual_line_sensors,
                              resolve_virtual_sensor_geometry,
                              virtual_sensor_is_active,
                              virtual_sensor_regions)


FORWARD_STATUS_PATH = "/dev/shm/obr_forward_line_status.json"


def component_labels_in_virtual_row(labels, row_geometry):
    """Coleta os componentes globais que realmente cruzam uma fileira virtual."""
    height, width = labels.shape
    visible_labels = set()
    for sensor_geometry in row_geometry.values():
        for region in virtual_sensor_regions(sensor_geometry):
            x0 = max(0, min(width, int(region["x0"])))
            y0 = max(0, min(height, int(region["y0"])))
            x1 = max(x0, min(width, int(region["x1"])))
            y1 = max(y0, min(height, int(region["y1"])))
            if x1 <= x0 or y1 <= y0:
                continue
            visible_labels.update(
                int(label)
                for label in np.unique(labels[y0:y1, x0:x1])
                if label != 0
            )
    return visible_labels


def bottom_fusion_path_is_connected(
    processed_line_mask,
    fusion_style_line,
    target_radius_px,
):
    """Exige que o target Fusion seja a mesma fita contínua em FAR e MEDIUM."""
    if (
        not isinstance(processed_line_mask, np.ndarray)
        or processed_line_mask.ndim != 2
        or processed_line_mask.size == 0
        or not isinstance(fusion_style_line, dict)
    ):
        return False
    far_point = fusion_style_line.get("farPoint")
    if not isinstance(far_point, dict):
        return False
    try:
        point_x = int(round(float(far_point["x"])))
        point_y = int(round(float(far_point["y"])))
        radius = max(0, int(target_radius_px))
    except (KeyError, TypeError, ValueError, OverflowError):
        return False
    height, width = processed_line_mask.shape
    if not 0 <= point_x < width or not 0 <= point_y < height:
        return False

    binary_mask = (processed_line_mask != 0).astype(np.uint8)
    if cv2.countNonZero(binary_mask) == 0:
        return False
    _, labels = cv2.connectedComponents(binary_mask, connectivity=8)
    geometry = resolve_virtual_sensor_geometry(processed_line_mask.shape)
    far_labels = component_labels_in_virtual_row(labels, geometry["far"])
    medium_labels = component_labels_in_virtual_row(labels, geometry["medium"])
    path_labels = far_labels.intersection(medium_labels)
    if not path_labels:
        return False

    x0 = max(0, point_x - radius)
    y0 = max(0, point_y - radius)
    x1 = min(width, point_x + radius + 1)
    y1 = min(height, point_y + radius + 1)
    target_labels = {
        int(label)
        for label in np.unique(labels[y0:y1, x0:x1])
        if label != 0
    }
    return bool(path_labels.intersection(target_labels))


def read_json_snapshot(path):
    """Lê um IPC atômico pequeno; ausência ou JSON inválido significam sem evidência."""
    try:
        with open(path, encoding="utf-8") as stream:
            text = stream.read(32769)
        if len(text) > 32768:
            return {}
        result = json.loads(text)
        return result if isinstance(result, dict) else {}
    except (OSError, ValueError):
        return {}


class GapValidator:
    """Limita a confirmação e o avanço; nunca calcula steering ou potência."""

    def __init__(self, config=None):
        self.config = dict(GAP_VALIDATION_CONFIG if config is None else config)
        self.reference = None
        self.near = {"present": False, "fresh": False, "lossConfirmed": False, "recovered": False}
        self.last_bottom = None
        self.last_near_present = None
        self.near_armed = False
        self.present_frames = 0
        self.missing_frames = 0
        self.bottom_far_present_frames = 0
        self.bottom_far_present = False
        self.bottom_fusion_candidate = False
        self.bottom_fusion_connected = False
        self.bottom_fusion_frames = 0
        self.bottom_fusion_ready = False
        self.forward_state = "UNCERTAIN"
        self.reset()

    def observe_near(self, mask, timestamp, sequence, wall_time, special=False,
                     bottom_far_present=False, virtual_sensors=None):
        """Confirma perda do NEAR-C em frames novos, sem aceitar pixels laterais."""
        cfg = NEAR_VIRTUAL_SENSOR_CONFIG
        height, width = mask.shape
        roi = (round(width * cfg["roi_x0"]), round(height * cfg["roi_y0"]),
               round(width * cfg["roi_x1"]), round(height * cfg["roi_y1"]))
        sensors = (
            virtual_sensors
            if isinstance(virtual_sensors, dict)
            else read_virtual_line_sensors(mask, "NENHUMA", False)
        )
        try:
            sensor_value = float(sensors.get("nearCenter", 0.0))
        except (TypeError, ValueError):
            sensor_value = 0.0
        if not math.isfinite(sensor_value):
            sensor_value = 0.0
        near_center_present = virtual_sensor_is_active(sensor_value)
        observed = {
            "state": "PRESENT" if near_center_present else "ABSENT",
            "present": near_center_present,
            "candidates": [],
            "roi": list(roi),
            "score": sensor_value,
            "sensorValue": sensor_value,
        }
        fresh = (type(sequence) is int and sequence > 0 and math.isfinite(timestamp)
                 and 0 <= wall_time - timestamp <= self.config["source_timeout"]
                 and (self.last_bottom is None or
                      (sequence > self.last_bottom[0] and timestamp > self.last_bottom[1])))
        if special:
            self.near_armed = False
            self.last_near_present = None
            self.present_frames = self.missing_frames = 0
            self.bottom_far_present_frames = 0
            self.bottom_far_present = False
        elif fresh:
            if self.last_bottom is not None and timestamp - self.last_bottom[1] > self.config["source_timeout"]:
                self.present_frames = self.missing_frames = 0
            if self.last_near_present is not None and wall_time - self.last_near_present > self.config["near_history_seconds"]:
                self.near_armed = False
            if observed["present"]:
                self.present_frames += 1
                self.missing_frames = 0
                self.bottom_far_present_frames = 0
                self.last_near_present = timestamp
                self.near_armed |= self.present_frames >= self.config["near_present_frames"]
            else:
                self.present_frames = 0
                self.missing_frames += 1
                self.bottom_far_present_frames = (
                    self.bottom_far_present_frames + 1
                    if bottom_far_present else 0
                )
            self.bottom_far_present = bool(bottom_far_present)
        if fresh:
            self.last_bottom = (sequence, timestamp)
        recent = self.last_near_present is not None and 0 <= wall_time - self.last_near_present <= self.config["near_history_seconds"]
        self.near = {**observed, "fresh": fresh,
                     "lossConfirmed": bool(fresh and not special and self.near_armed and recent
                                           and self.missing_frames >= self.config["near_loss_frames"]),
                     "bottomFarConfirmed": bool(
                         fresh and not special and not observed["present"]
                         and self.bottom_far_present_frames
                         >= self.config["bottom_far_present_frames"]
                     ),
                     "recovered": bool(fresh and not special and observed["present"]
                                       and self.present_frames >= GEOMETRIC_GAP_REACQUIRE_FRAMES)}
        return self.near

    def reset(self):
        self.decision = "NORMAL"
        self.started = None
        self.last_forward_present = None
        self.last_forward = None
        self.forward_present_count = 0
        self.bottom_fusion_candidate = False
        self.bottom_fusion_connected = False
        self.bottom_fusion_frames = 0
        self.bottom_fusion_ready = False
        self.reason = "BOTTOM_AUTHORITY"

    @staticmethod
    def finite_field(values, *names):
        """Confirma que ao menos um campo existe e contém uma medida finita."""
        if not isinstance(values, dict):
            return False
        for name in names:
            try:
                if math.isfinite(float(values.get(name))):
                    return True
            except (TypeError, ValueError):
                pass
        return False

    def observe_bottom_fusion(
        self,
        fusion_style_line,
        virtual_sensors,
        fresh,
        gap_active,
        special,
        processed_line_mask=None,
    ):
        """Confirma em frames novos uma trajetória Fusion ainda distante do NEAR."""
        if special or not gap_active:
            self.bottom_fusion_candidate = False
            self.bottom_fusion_connected = False
            self.bottom_fusion_frames = 0
            self.bottom_fusion_ready = False
            return False
        if not fresh:
            # Frame repetido não mantém autoridade nem renova a evidência GAP.
            self.bottom_fusion_ready = False
            return False

        far_trusted = (
            isinstance(virtual_sensors, dict)
            and virtual_sensors.get("farTrusted") is True
            and self.finite_field(
                virtual_sensors,
                "farBandPosition",
                "farPosition",
            )
        )
        medium_trusted = (
            isinstance(virtual_sensors, dict)
            and virtual_sensors.get("mediumTrusted") is True
            and self.finite_field(virtual_sensors, "mediumPosition")
        )
        selection = (
            fusion_style_line.get("selection")
            if isinstance(fusion_style_line, dict)
            else None
        )
        basic_geometry = bool(
            isinstance(fusion_style_line, dict)
            and fusion_style_line.get("valid") is True
            and selection in ("nearCenter", "deepestFallback")
            and self.finite_field(fusion_style_line, "angleDeg")
            and isinstance(fusion_style_line.get("nearPoint"), dict)
            and isinstance(fusion_style_line.get("farPoint"), dict)
            and far_trusted
            and medium_trusted
        )
        self.bottom_fusion_connected = bool(
            basic_geometry
            and bottom_fusion_path_is_connected(
                processed_line_mask,
                fusion_style_line,
                self.config["bottom_fusion_target_radius_px"],
            )
        )
        plausible = bool(basic_geometry and self.bottom_fusion_connected)
        try:
            stable_frames = int(fusion_style_line.get("targetStableFrames", 0))
        except (AttributeError, TypeError, ValueError):
            stable_frames = 0

        self.bottom_fusion_candidate = plausible
        if not plausible:
            self.bottom_fusion_frames = 0
        elif stable_frames <= 1:
            # Uma troca brusca de target reinicia a estabilidade Fusion em um.
            self.bottom_fusion_frames = 1
        else:
            self.bottom_fusion_frames += 1
        self.bottom_fusion_ready = bool(
            plausible
            and stable_frames >= self.config["bottom_fusion_reacquire_frames"]
            and self.bottom_fusion_frames
            >= self.config["bottom_fusion_reacquire_frames"]
        )
        return self.bottom_fusion_ready

    def process_frame(self, mask, maneuver, controller, sequence, timestamp,
                      reading, now, wall_time, preferred_direction=None,
                      sensor_recovery_requested=False, bottom_far_present=False,
                      fusion_style_line=None, virtual_sensors=None):
        """Aplica o gate local e a reaquisição nativa sem recalcular o controle normal."""
        special = (maneuver.green_direction != "NENHUMA" or sensor_recovery_requested
                   or controller.virtual_turn_tracker.state != "NORMAL"
                   or controller.pivot_state_tracker.state != "NONE"
                   or controller.medium_spin_tracker.state != "NONE")
        # O FAR inteiro e a FAR BAND representam a mesma continuação distante.
        # Qualquer posição trusted mantém GAP; a chamada explícita permanece
        # apenas para compatibilidade com testes e consumidores antigos.
        bottom_far_present = bool(
            bottom_far_present
            or (
                isinstance(virtual_sensors, dict)
                and virtual_far_line_is_visible(virtual_sensors)
            )
        )
        near = self.observe_near(
            mask, timestamp, sequence, wall_time, special,
            bottom_far_present=bottom_far_present,
            virtual_sensors=virtual_sensors,
        )
        gap_active = bool(
            maneuver.gap_forward_active
            or maneuver.gap_fusion_reacquire_active
        )
        candidate = gap_entry_is_required(
            gap_active, maneuver.green_direction,
            near_line_present=near["present"], near_loss_confirmed=near["lossConfirmed"],
            special_control=special)
        if candidate and not gap_active:
            maneuver.gap_forward_active = True
            maneuver.gap_fusion_reacquire_active = False
            maneuver.gap_forward_frames = maneuver.gap_reacquire_frames = 0
            maneuver.gap_line_lost_seen = True
            gap_active = True
        bottom_fusion_ready = self.observe_bottom_fusion(
            fusion_style_line,
            virtual_sensors,
            near["fresh"],
            gap_active,
            special,
            processed_line_mask=mask,
        )
        if maneuver.gap_forward_active and near["fresh"] and not special:
            # Uma interrupção entre capturas não conta como presença consecutiva.
            if self.present_frames < 2:
                maneuver.gap_reacquire_frames = 0
            state = update_gap_forward_recovery(
                True, maneuver.gap_forward_frames, maneuver.gap_reacquire_frames,
                maneuver.gap_line_lost_seen, near["present"])
            maneuver.gap_forward_active = state["active"]
            maneuver.gap_forward_frames = state["forwardFrames"]
            maneuver.gap_reacquire_frames = state["reacquireFrames"]
            maneuver.gap_line_lost_seen = state["lineLostSeen"]
            if not maneuver.gap_forward_active:
                controller.line_search_tracker.stop()
        self.apply(maneuver, controller, candidate, near["recovered"], special,
                   reading, now, wall_time, preferred_direction,
                   bottom_far_confirmed=near["bottomFarConfirmed"],
                   bottom_fusion_confirmed=bottom_fusion_ready)
        if self.decision in ("CHECKING", "GAP") and bottom_fusion_ready:
            # O FAR guia a travessia com correção limitada até que FAR e MEDIUM
            # confirmem juntos a geometria completa escolhida pelo Fusion.
            maneuver.gap_forward_active = False
            maneuver.gap_fusion_reacquire_active = True
            controller.line_search_tracker.stop()
        elif maneuver.gap_fusion_reacquire_active:
            maneuver.gap_fusion_reacquire_active = False
            if self.decision in ("CHECKING", "GAP"):
                maneuver.gap_forward_active = True
        # CHECKING/GAP nunca inicia busca pelo contador legado. A busca só recebe
        # autoridade quando apply() classifica a perda como LOST.
        return False

    def remember(self, mask, fusion, command, timestamp, sequence):
        """Renova a referência somente durante seguimento inferior normal observado."""
        if (command.get("lineState") == "LINE" and command.get("virtualState") == "NORMAL"
                and command.get("controlSource") in ("fusion", "virtual")
                and self.decision == "NORMAL"):
            reference = bottom_reference(mask, fusion, timestamp, sequence)
            if reference is not None:
                self.reference = reference

    def present_reading(self, reading, wall_time):
        """Confirma fita frontal presente e recente, sem exigir extrapolação inferior."""
        try:
            cfg = self.config
            timestamp = float(reading["forwardLineTimestamp"])
            confidence = float(reading["forwardPathConfidence"])
            sequence = reading["forwardLineSequence"]
            if (type(sequence) is not int or sequence <= 0 or not math.isfinite(timestamp)
                    or not 0 <= wall_time - timestamp <= cfg["source_timeout"]):
                return False, None
            valid = (reading.get("forwardPathVersion") == 2
                     and reading.get("forwardPathState") == "PRESENT"
                     and reading.get("forwardLinePresent") is True
                     and reading.get("forwardLineVisible") is True
                     and math.isfinite(confidence) and confidence == 1.0)
            return valid, (sequence, timestamp)
        except (KeyError, TypeError, ValueError):
            return False, None

    def update(self, candidate, recovered, special, reading, now, wall_time,
               bottom_far_confirmed=False, bottom_fusion_confirmed=False):
        """Confirma GAP e limita avanço sem continuação inferior observável."""
        present, stamp = self.present_reading(reading, wall_time)
        state = reading.get("forwardPathState")
        self.forward_state = (state if reading.get("forwardPathVersion") == 2
                              and state in ("PRESENT", "UNCERTAIN", "ABSENT")
                              and stamp is not None and 0 <= wall_time - stamp[1] <= self.config["source_timeout"]
                              else "UNCERTAIN")
        if special or recovered:
            self.reset()
            if special:
                self.reference = None
            return self.decision
        if self.started is None:
            if not candidate:
                return self.decision
            self.started = now
            self.decision = "CHECKING"
        if self.decision == "LOST":
            if not (bottom_far_confirmed or bottom_fusion_confirmed):
                return self.decision
            # O FAR pode entrar no campo depois que a janela inicial termina.
            # Dois frames trusted cancelam a busca e retomam a travessia sem
            # exigir que o operador empurre a fita até o NEAR-C.
            self.decision = "GAP"
            self.reason = "BOTTOM_FAR_RECOVERED_AFTER_LOST"
        bottom_continuation = bool(
            bottom_far_confirmed or bottom_fusion_confirmed
        )
        if bottom_continuation:
            # A evidência inferior pertence ao mesmo frame da máscara e pode
            # renovar GAP antes de o prazo frontal vencer neste ciclo.
            self.last_forward_present = now
        elapsed = now - self.started
        if (
            elapsed < 0
            or (
                elapsed >= self.config["max_gap_seconds"]
                and not bottom_continuation
            )
        ):
            self.decision, self.reason = "LOST", "GAP_TIME_LIMIT"
            return self.decision
        # A evidência inferior sincronizada já foi consumida. A frontal continua
        # sujeita aos prazos e não pode reabrir uma janela que já venceu.
        if self.decision == "CHECKING" and elapsed >= self.config["confirmation_seconds"]:
            self.decision, self.reason = "LOST", "NO_CONFIRMED_CONTINUATION"
            return self.decision
        if self.decision == "GAP" and (self.last_forward_present is None or
                                       now - self.last_forward_present >= self.config["evidence_grace_seconds"]):
            self.decision, self.reason = "LOST", "CONTINUATION_EXPIRED"
            return self.decision
        if bottom_far_confirmed:
            # O FAR inferior é sincronizado com a perda local. Em um GAP curto,
            # ele pode enxergar a continuação antes de a frontal confirmar.
            self.decision = "GAP"
            if self.reason != "BOTTOM_FAR_RECOVERED_AFTER_LOST":
                self.reason = "BOTTOM_FAR_PRESENT"
        if bottom_fusion_confirmed:
            self.decision, self.reason = "GAP", "BOTTOM_FUSION_REACQUIRED"
        if stamp != self.last_forward:
            increasing = self.last_forward is None or (stamp is not None
                                                       and stamp[0] > self.last_forward[0]
                                                       and stamp[1] > self.last_forward[1])
            self.forward_present_count = self.forward_present_count + 1 if present and increasing else 0
            # Um IPC ausente ou fora de ordem não torna um frame já consumido novo.
            if stamp is not None and increasing:
                self.last_forward = stamp
            if present and increasing:
                self.last_forward_present = now
        if (self.decision != "GAP" and present
                and self.forward_present_count >= self.config["forward_present_frames"]):
            self.decision, self.reason = "GAP", "FORWARD_TAPE_PRESENT"
        elif self.decision == "CHECKING":
            self.reason = "WAITING_FORWARD_CONFIRMATION"
        return self.decision

    def apply(self, maneuver, controller, candidate, near_recovered, special,
              reading, now, wall_time, preferred_direction=None,
              bottom_far_confirmed=False, bottom_fusion_confirmed=False):
        """Muda só o gate GAP/perda e aciona o tracker de recovery que já existe."""
        previous_decision = self.decision
        decision = self.update(candidate, near_recovered,
                               special, reading, now, wall_time,
                               bottom_far_confirmed=bottom_far_confirmed,
                               bottom_fusion_confirmed=bottom_fusion_confirmed)
        if special:
            maneuver.gap_forward_active = False
            maneuver.gap_fusion_reacquire_active = False
            return decision
        if decision == "NORMAL" and previous_decision == "LOST" and near_recovered:
            # O reencontro confirmado no NEAR-C devolve autoridade ao controle
            # normal sem esperar o tracker de busca aceitar pixels laterais.
            controller.line_search_tracker.stop()
        if (decision in ("CHECKING", "GAP")
                and not maneuver.gap_forward_active
                and not maneuver.gap_fusion_reacquire_active):
            maneuver.gap_forward_active = True
            maneuver.gap_forward_frames = 0
            maneuver.gap_reacquire_frames = 0
            maneuver.gap_line_lost_seen = True
            controller.line_search_tracker.stop()
        elif decision == "LOST":
            maneuver.gap_forward_active = False
            maneuver.gap_fusion_reacquire_active = False
            maneuver.gap_forward_frames = 0
            maneuver.gap_reacquire_frames = 0
            maneuver.gap_recent_near_frames = 0
            # Ré e varredura são as existentes. A frontal não escolhe seu lado.
            controller.line_search_tracker.start(
                preferred_direction, backup_frames=VIRTUAL_BLIND_SEARCH_BACKUP_FRAMES)
        elif decision == "NORMAL":
            maneuver.gap_forward_active = False
            maneuver.gap_fusion_reacquire_active = False
        return decision

    def diagnostics(self):
        return {"gapValidationDecision": self.decision, "gapValidationReason": self.reason,
                "nearLinePresent": self.near["present"],
                "nearLineState": ("PRESENT" if self.near["present"] else "LOST") if self.near["fresh"] else "UNKNOWN",
                "nearLineMissingFrames": self.missing_frames,
                "bottomFarLinePresent": bool(
                    self.near.get("fresh") and self.bottom_far_present
                ),
                "bottomFarPresentFrames": self.bottom_far_present_frames,
                "bottomFusionReacquireCandidate": self.bottom_fusion_candidate,
                "bottomFusionPathConnected": self.bottom_fusion_connected,
                "bottomFusionReacquireFrames": self.bottom_fusion_frames,
                "bottomFusionReacquireReady": self.bottom_fusion_ready,
                "forwardPresenceState": self.forward_state,
                "bottomPathReference": self.reference}
