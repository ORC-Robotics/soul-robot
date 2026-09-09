"""Confirma GAP com evidência frontal e devolve perdas ao recovery inferior."""

import json
import math

from .camera_config import (GAP_VALIDATION_CONFIG, NEAR_LINE_PRESENCE_CONFIG,
                            VIRTUAL_BLIND_SEARCH_BACKUP_FRAMES, GEOMETRIC_GAP_REACQUIRE_FRAMES)
from .forward_path import bottom_reference
from .line_presence import analyze_line_presence
from .line_control import gap_entry_is_required, update_gap_forward_recovery


FORWARD_STATUS_PATH = "/dev/shm/obr_forward_line_status.json"


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
        self.bottom_fusion_frames = 0
        self.bottom_fusion_ready = False
        self.forward_state = "UNCERTAIN"
        self.reset()

    def observe_near(self, mask, timestamp, sequence, wall_time, special=False,
                     bottom_far_present=False):
        """Confirma perda física com frames novos; Fusion e NEAR-C não votam aqui."""
        cfg = NEAR_LINE_PRESENCE_CONFIG
        height, width = mask.shape
        roi = (round(width * cfg["roi_x0"]), round(height * cfg["roi_y0"]),
               round(width * cfg["roi_x1"]), round(height * cfg["roi_y1"]))
        observed = analyze_line_presence(mask, roi, cfg)
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
    ):
        """Confirma em frames novos uma trajetória Fusion ainda distante do NEAR."""
        if special or not gap_active:
            self.bottom_fusion_candidate = False
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
        plausible = bool(
            isinstance(fusion_style_line, dict)
            and fusion_style_line.get("valid") is True
            and selection in ("nearCenter", "deepestFallback")
            and self.finite_field(fusion_style_line, "angleDeg")
            and isinstance(fusion_style_line.get("nearPoint"), dict)
            and isinstance(fusion_style_line.get("farPoint"), dict)
            and (far_trusted or medium_trusted)
        )
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
        near = self.observe_near(
            mask, timestamp, sequence, wall_time, special,
            bottom_far_present=bottom_far_present,
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
            # O Fusion assume com sua geometria completa; FAR/MEDIUM não geram
            # direção diretamente e continuam apenas validando o target escolhido.
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
        """Uma perda isolada espera; evidência repetida confirma GAP dentro de um teto."""
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
            return self.decision
        elapsed = now - self.started
        if elapsed < 0 or elapsed >= self.config["max_gap_seconds"]:
            self.decision, self.reason = "LOST", "GAP_TIME_LIMIT"
            return self.decision
        # Verifica os prazos antes de ler evidência nova. Um ciclo atrasado não
        # pode renovar uma janela de avanço que já venceu.
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
            self.last_forward_present = now
            self.decision, self.reason = "GAP", "BOTTOM_FAR_PRESENT"
        if bottom_fusion_confirmed:
            self.last_forward_present = now
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
            # Presença física confirmada libera também o Fusion lateral, que
            # não precisa esperar o tracker de busca aceitar pixels soltos.
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
                "bottomFusionReacquireFrames": self.bottom_fusion_frames,
                "bottomFusionReacquireReady": self.bottom_fusion_ready,
                "forwardPresenceState": self.forward_state,
                "bottomPathReference": self.reference}
