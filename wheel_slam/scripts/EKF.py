import numpy as np
#Based on AtsushiSakai PythonRobotics EKF


class EKF:
    def __init__(self, Q, R):
        self.Q = Q 
        self.R = R 

    def f(self, x: np.ndarray, u:np.ndarray) -> np.ndarray:
        """! add the odometry u to the robot pose state x.

        "@param x: state vector
         @param u: odometry vector
        
        """

