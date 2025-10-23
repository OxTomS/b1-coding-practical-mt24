class controller:
    def __init__(self, kp: float = 0.05, kd: float = 0.733, dt: float = 0.01):
        self.kp = kp
        self.kd = kd
        self.dt = dt
        self.prev_error = None

    def compute_control(self, reference: float, current_state: float, current_velocity: float) -> float:
        error = reference - current_state
        
        if self.prev_error is None:
            self.prev_error = error
        
        u = self.kp * error + self.kd * (error - self.prev_error)
        self.prev_error = error
        
        return u

    def reset(self):
        self.prev_error = None