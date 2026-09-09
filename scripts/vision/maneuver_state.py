"""Estado persistente das manobras GREEN, GAP e Fusion."""

class LineManeuverState:
    """Possui exclusivamente o estado persistente de GREEN, GAP e Fusion."""

    def __init__(self):
        self.fusion_target_history = None
        self.green_direction = "NENHUMA"
        self.green_curve_started = False
        self.green_centered_frames = 0
        self.green_active_frames = 0
        self.gap_forward_active = False
        self.gap_fusion_reacquire_active = False
        self.gap_forward_frames = 0
        self.gap_reacquire_frames = 0
        self.gap_line_lost_seen = False
        self.gap_recent_near_frames = 0
        self.green_armed = True
        self.green_clear_frames = 0
        self.last_applied_fusion_command = None
        self.green_candidate_hold_frames = 0
        self.green_candidate_hold_blocked = False
        self.last_green_fusion_line = None
        self.green_fusion_target_missing_frames = 0
