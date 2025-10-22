class controller:
    def __init__(self):
        """
        Initialize PD controller with proportional and derivative gains
        Args:
            kp (float): Proportional gain
            kd (float): Derivative gain
        """
        self.kp = 0.15  # Proportional gain
        self.kd = 0.6  # Derivative gain
        self.prev_error = 0  # Store previous error for derivative term
        self.dt = 1  # Time step (can be adjusted as needed)

    def compute_control(self, reference, current_state, current_velocity):
        """
        Compute control output using PD control law
        Args:
            reference (float): Desired setpoint
            current_state (float): Current system state
            current_velocity (float): Current system velocity
        Returns:
            float: Control output
        """
        # Calculate error
        error = reference - current_state
        
        # Calculate error derivative (rate of change)
        error_derivative = (error - self.prev_error) / self.dt
        
        # Store current error for next iteration
        self.prev_error = error
        
        # PD control law: u = Kp*e + Kd*de/dt
        control_output = self.kp * error + self.kd * error_derivative
        
        return control_output

    def reset(self):
        """Reset controller internal states"""
        self.prev_error = 0