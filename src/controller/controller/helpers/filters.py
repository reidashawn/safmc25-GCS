
class RangedFilter:
    def __init__(self, length, range):
        self.length = length
        self.range = range
        self.array = []
    def update(self, new_value):
        if len(self.array) < self.length:
            self.array.append(new_value)
        else:
            self.array.pop(0)
            self.array.append(new_value)
        min, max = 1000, 0
        for i in self.array:
            if i < min:
                min = i
            elif i > max:
                max = i
        # print(max-min)
        if 0 < (max - min) < self.range:
            return new_value
        else:
            return None


class LowPassFilter:
    def __init__(self, alpha, initial_value=None):
        """
        Initializes the low-pass filter.

        Args:
            alpha (float): Smoothing factor (0 < alpha <= 1). 
                           Lower values mean stronger filtering.
            initial_value (float): Initial output value.
        """
        self.alpha = alpha
        self.filtered_value = initial_value

    def update(self, new_value):
        """
        Updates the filter with a new input value.

        Args:
            new_value (float): The new input value.

        Returns:
            float: The filtered value.
        """

        if self.filtered_value is None:
            self.filtered_value = new_value
        else:
            self.filtered_value = self.alpha * new_value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value


class HighPassFilter:
    def __init__(self, alpha, initial_value=None):
        """
        Initializes the high-pass filter.

        Args:
            alpha (float): Smoothing factor (0 < alpha <= 1).
                           Lower values mean stronger filtering.
            initial_value (float): Initial input and output values.
        """
        self.alpha = alpha
        self.previous_input = initial_value
        self.filtered_value = 0

    def update(self, new_value):
        """
        Updates the filter with a new input value.

        Args:
            new_value (float): The new input value.

        Returns:
            float: The filtered value.
        """
        if self.previous_input is None:
            self.previous_input = new_value
        else:
            self.filtered_value = self.alpha * (self.filtered_value + new_value - self.previous_input)
        self.previous_input = new_value
        return self.filtered_value
