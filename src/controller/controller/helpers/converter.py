import numpy as np

class LocalToNEDConverter:
    def __init__(self, heading: float):
        """
        Initialize the converter with a heading (in degrees).
        """
        if heading is not None:
            self.heading = np.radians(heading)  # Convert to radians
        else:
            self.heading = None
    
    def set_heading(self, heading):
        if heading is not None:
            self.heading = np.radians(heading)  # Convert to radians
        else:
            self.heading = None
    
    def transform_to_ned(self, local_vector):
        """
        Transforms a vector from the local frame to the NED frame.
        
        :param local_vector: A tuple or list (x_local, y_local)
        :return: A tuple (x_ned, y_ned)
        """
        
        if self.heading is None:
            return None

        # Rotation matrix from local frame to NED
        R = np.array([
            [np.cos(self.heading), -np.sin(self.heading)],
            [np.sin(self.heading),  np.cos(self.heading)]
        ])
        
        # Apply transformation
        ned_vector = R @ np.array(local_vector)
        
        return tuple(ned_vector)

converter = LocalToNEDConverter(250.0)
print(converter.transform_to_ned([.7,.5]))
