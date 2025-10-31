class PID:
    def __init__(self, Kp = 0.0, Ki = 0.0, Kd = 0.0, bias = 0.0, max_integral = None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.bias = bias
        self.max_integral = max_integral

        self.set_point = 0.
        self.prev_error = 0.

        self.integral = 0.

    def update(self, feedback):
        error = self.set_point - feedback

        if (self.max_integral is None):
            self.integral += self.Ki * error
        elif (abs(self.integral + self.Ki * error) <= self.max_integral):
            self.integral += self.Ki * error

        control_signal = self.Kp * error + self.integral + self.Kd * (error - self.prev_error) + self.bias

        return control_signal 
    
    def set_PID_gains(self, Kp, Ki, Kd, bias = 0.):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.bias = bias