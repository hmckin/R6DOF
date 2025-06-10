class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits
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
        # Clamp output
        min_out, max_out = self.output_limits
        if min_out is not None:
            output = max(min_out, output)
        if max_out is not None:
            output = min(max_out, output)
        self.prev_error = error
        return output
