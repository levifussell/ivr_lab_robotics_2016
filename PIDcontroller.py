
class PIDController:

    def __init__(self, k_proportional=1, k_integral=0, k_derivative=0):
        self.k_proportional = k_proportional
        self.k_integral = k_integral
        # self.kI_max = 2
        # self.kI_min = 0.0001
        self.k_derivative = k_derivative
        self.error_sum = 0
        self.previous_position = 0

    def updatePosition(self, plant_error, plant_curr_position):

        # calculate proportional value
        p_val = self.k_proportional * plant_error

        # calculate integral value
        self.error_sum += plant_error
        i_val = self.k_integral * self.error_sum

        # calculate derivative value
        d_val = self.k_derivative * (plant_curr_position - self.previous_position)
        self.previous_position = plant_curr_position

        return p_val + i_val + d_val
