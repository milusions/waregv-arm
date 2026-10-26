import math
import matlab.engine

class Planar3DOFArm:
    def __init__(self, l1, l2, l3, x_min=0.0, y_min=-50.0, joint_limits=None, eng=None):
        # joint_limits: [(min, max), (min, max), (min, max)] in radians for theta1..3.
        # Default is +/-120 deg; the app replaces it with the limits read from the servos.
        lim = math.radians(120)
        self.joint_limits = joint_limits or [(-lim, lim)] * 3
        self.L1 = float(l1)
        self.L2 = float(l2)
        self.L3 = float(l3)
        self.x_min = float(x_min)
        self.y_min = float(y_min)

        # Initialize or assign MATLAB Engine instance
        if eng is None:
            self.eng = matlab.engine.start_matlab()
        else:
            self.eng = eng

    def _build_dh_table(self, theta1=0.0, theta2=0.0, theta3=0.0):
        """Constructs DH parameters matrix [theta, d, a, alpha] for MATLAB."""
        dh_list = [
            [float(theta1), 0.0, self.L1, 0.0],
            [float(theta2), 0.0, self.L2, 0.0],
            [float(theta3), 0.0, self.L3, 0.0]
        ]
        return matlab.double(dh_list)

    def forward_kinematics(self, theta1, theta2, theta3):
        dh_table = self._build_dh_table(theta1, theta2, theta3)
        
        # Heavy computation performed entirely by MATLAB
        res = self.eng.kinematics(dh_table)
        
        x = float(res[0][0])
        y = float(res[0][1])
        phi = float(res[0][2])
             
        return x, y, phi

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def inverse_kinematics(self, x, y, phi, elbow_up=True):
        dh_table = self._build_dh_table()
        target_pose = matlab.double([float(x), float(y), float(phi)])

        try:
            # Heavy computation and kinematic geometric checks performed by MATLAB
            res = self.eng.inverse_kinematics(
                dh_table, 
                target_pose, 
                bool(elbow_up), 
                float(self.x_min), 
                float(self.y_min)
            )
        except matlab.engine.MatlabExecutionError as err:
            # Re-raise MATLAB errors as Python ValueErrors to maintain interface behavior
            raise ValueError(str(err)) from err

        theta1 = float(res[0][0])
        theta2 = float(res[0][1])
        theta3 = float(res[0][2])

        # Validate joint limits on returned angles in Python
        for i, th in enumerate((theta1, theta2, theta3), 1):
            lo, hi = self.joint_limits[i - 1]
            if not (lo - 1e-9 <= th <= hi + 1e-9):
                raise ValueError(f"Joint {i} limit exceeded: {math.degrees(th):.1f}° not in "
                                 f"[{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]")

        return theta1, theta2, theta3