class controller:
    """
    PD-style controller that uses position error (proportional) and velocity error (derivative-like).
    control = Kp * (ref_pos - pos) + Kv * (ref_vel - vel)
    """
    def __init__(self, kp: float = 0.15, kv: float = 0.6, dt: float | None = None):
        self.kp = kp
        self.kv = kv
        self.dt = dt

    def compute_control(
        self,
        reference_pos: float,
        current_pos: float,
        current_vel: float,
        reference_vel: float = 0.0,
        dt: float | None = None
    ) -> float:
        """
        Compute a control action to drive the submarine toward reference_pos and reference_vel.
        - reference_pos: desired depth
        - current_pos: measured depth
        - current_vel: measured vertical velocity
        - reference_vel: optional desired vertical velocity (default 0)
        - dt: optional timestep (not required for this formulation but accepted)
        Returns a scalar control action.
        """
        # choose dt if needed for callers; not used in this simple law but accepted for compatibility
        if dt is None:
            dt = self.dt

        pos_error = reference_pos - current_pos
        vel_error = reference_vel - current_vel

        # PD-like output: P on position error, D-like on velocity error
        u = self.kp * pos_error + self.kv * vel_error
        return float(u)

    def reset(self):
        """No internal state required for this implementation, kept for API compatibility."""
        return