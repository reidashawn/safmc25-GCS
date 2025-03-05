class Joystick:
    def __init__(self, zero, dead_zone, max_input, min_input, max_output, min_output = None):
        self.zero = zero
        self.dead_zone = dead_zone
        self.max_input = max_input
        self.min_input = min_input
        self.max_output = max_output
        self.min_output = 0 - (max_output - 0) if not min_output else min_output
        # print(f"Min_output: {self.min_output}, Max_output: {self.max_output}, input: {min_output}")
    
    def get_output(self, input):
        # Using ROS 2 logging instead of print statements for visibility
        # self.get_logger().info(f"min_input: {self.min_input}, input: {input}, max_input: {self.max_input}")

        if input > self.max_input:
            return self.max_output
        elif input <= self.min_input:
            return self.min_output
        elif input < (self.zero - self.dead_zone / 2):  # Below zero deadzone
            # Log scaling output below deadzone
            # print(f"Scaling output below deadzone: -({self.min_output}) * ({input} - {self.min_input}) / ({self.zero} - {self.min_input}) = {-(self.min_output) * (input - self.min_input) / (self.zero - self.min_input)}")
            return (self.min_output) * (self.zero - input ) / (self.zero - self.min_input)  # Fix scaling
        elif input > (self.zero + self.dead_zone / 2):  # Above zero deadzone
            # Log scaling output above deadzone
            # self.get_logger().info(f"Scaling output above deadzone: {(self.max_output) * (input - self.zero) / (self.max_input - self.zero)}")
            return (self.max_output) * (input - self.zero) / (self.max_input - self.zero)  # Fix scaling
        else:  # Deadzone region
            return 0

    def recenter(self, center, min_input=None, max_input=None):
        self.zero = center
        self.min_input = min_input if min_input is not None else self.min_input
        self.max_input = max_input if max_input is not None else self.max_input

        return True
