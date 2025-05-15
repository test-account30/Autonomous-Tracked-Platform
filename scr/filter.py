class HighPassFilter:
    def __init__(self, alpha=0.8, threshold=0.1):
        self.filtered_acceleration = [0, 0, 0]
        self.alpha = alpha  # Smoothing factor
        self.threshold = threshold  # Threshold for noise removal

    def apply_high_pass_filter(self, raw_acceleration):
        # Apply high-pass filter to the raw acceleration data
        self.filtered_acceleration[0] = self.alpha * (self.filtered_acceleration[0] + raw_acceleration[0])
        self.filtered_acceleration[1] = self.alpha * (self.filtered_acceleration[1] + raw_acceleration[1])
        self.filtered_acceleration[2] = self.alpha * (self.filtered_acceleration[2] + raw_acceleration[2])

        # Check if the filtered acceleration exceeds the threshold
        if (abs(self.filtered_acceleration[0]) > self.threshold or
                abs(self.filtered_acceleration[1]) > self.threshold or
                abs(self.filtered_acceleration[2]) > self.threshold):
            # If the change is significant, consider it as non-noise and use the filtered acceleration
            return self.filtered_acceleration
        else:
            # If the change is below the threshold, consider it as noise and return [0, 0, 0] or the raw data
            return [0, 0, 0]  # You may also return raw_acceleration instead of [0, 0, 0]
