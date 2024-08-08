

"""Simple PID controller to keep the person tracked within the camera's field"""
class PIDPoint():
    def __init__(self,setpoint):
        self.setpoint = setpoint #(x,y)
        self.prev_error_x = 0
        self.integral_error_x = 0
        self.derivative_error_x = 0
        self.error_x = 0

        self.prev_error_y = 0
        self.integral_error_y = 0
        self.derivative_error_y = 0
        self.error_y = 0
        
        self.kp = 1
        self.ki = 1
        self.kd = 1
        self.time_step = 1 #frame
        self.output = 0 #correction to apply in the form (output_x, output_y)

    def compute(self,pos)->(float,float):
        """Funtion to compute the output of the PID controller, given the current
        position 'pos'. In our case pos represent the current coordinates of the middlepoint of tracked person"""
        if self.integral_error_x > 3 :
            self.integral_error_x = 0
        if self.integral_error_y > 3:
            self.integral_error_y = 0
        setpoint_x, setpoint_y = self.setpoint
        pos_x = pos.x
        pos_y = pos.y
        self.error_x = setpoint_x - pos_x
        self.error_y = setpoint_y - pos_y

        self.integral_error_x = self.integral_error_x + self.error_x * self.time_step
        self.derivative_error_x = (self.error_x - self.prev_error_x) * self.time_step
        self.prev_error_x = self.error_x

        self.integral_error_y = self.integral_error_y + self.error_y * self.time_step
        self.derivative_error_y = (self.error_y - self.prev_error_y) * self.time_step
        self.prev_error_y = self.error_y

        self.output = (self.kp * self.error_x + self.ki * self.integral_error_x + self.kd * self.derivative_error_x, self.kp * self.error_y + self.ki * self.integral_error_y + self.kd * self.derivative_error_y)
        return self.output


"""PID controller to ensure that the drone's distance to the tracked person is within a certain field"""
class PIDRange():
    def __init__(self,set_range_min, set_range_max):
        self.setrange = (set_range_min,set_range_max)
        self.error = 0
        self.prev_error = 0
        self.kp = 1
        self.ki = 1
        self.kd = 1
        self.time = 1 #frame

"""
class Point():
    def __init__(self, x, y):
        self.x = x
        self.y = y

def main():
    test_input = [Point(0.3,0.2),Point(0.5,0.5),Point(0.7,0.3),Point(0.8,0.5),Point(1,0.7),Point(0.5,0)]
    #test_input = [Point(0.3,0.3),Point(0.5,0.5),Point(0.7,0.7),Point(0.5,0.5),Point(1,1),Point(0,0)]
    test_pid = PIDPoint((0.5,0.5))
    test_correction = map(test_pid.compute, test_input)
    print(list(test_correction))

main()

"""