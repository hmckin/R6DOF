class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(None, None)):
        self.kp = kp # Proportional gain, responds to the current error with a magnitude proportional to the error
        self.ki = ki # Integral gain, cumulative error over time to address residual steady-state error
        self.kd = kd # Derivative gain, anticipates future error by considering the rate of change of the error
        self.setpoint = setpoint # Desired value
        self.output_limits = output_limits # Output limits
        self.clear() 

    def clear(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, measurement, dt):
        error = self.setpoint - measurement
        if self.prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.prev_error) / dt
        self.integral += error * dt
        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )
        # Clamp output if beyond physical limits
        min_out, max_out = self.output_limits
        if min_out is not None:
            output = max(min_out, output)
        if max_out is not None:
            output = min(max_out, output)
        self.prev_error = error
        return output # computed control signal to adjust the system
