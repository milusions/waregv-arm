import os
import math
import matlab.engine
import io


    
class Planar3DOFArm:
    def __init__(self, l1, l2, l3, x_min=0.0, y_min=-50.0, joint_limits=None, script_dir=None):
        # joint_limits: [(min, max), (min, max), (min, max)] in radians for theta1..3.
        lim = math.radians(120)
        self.joint_limits = joint_limits or [(-lim, lim)] * 3
        self.L1 = float(l1)
        self.L2 = float(l2)
        self.L3 = float(l3)
        self.x_min = float(x_min)
        self.y_min = float(y_min)
        self.out = io.StringIO()
        self.err = io.StringIO()

        # Initialize MATLAB engine
        print("Connecting to MATLAB engine...")
        self.eng = matlab.engine.start_matlab()
        print("Successfully connected to MATLAB engine!")

        # Add path containing kinematics.m and inverse_kinematics.m
        current_dir = os.getcwd()
        folder = os.path.join(current_dir,script_dir) if script_dir else os.path.join(current_dir,"matlab")
        self.eng.addpath(folder, nargout=0)

    def forward_kinematics(self, theta1, theta2, theta3):
        # Build 3x3 DH table matrix for MATLAB: [theta, d, a, alpha]
        dh_table = matlab.double([
            [float(theta1), 0.0, self.L1, 0.0],
            [float(theta2), 0.0, self.L2, 0.0],
            [float(theta3), 0.0, self.L3, 0.0]
        ])

        # Call kinematics.m
        result = self.eng.kinematics(dh_table, nargout=1,stdout=self.out, stderr=self.err)
        pos = result[0]
        return float(pos[0]), float(pos[1]), float(pos[2])

    def inverse_kinematics(self, x, y, phi, elbow_up=True):
        # Build 3x3 DH table matrix for MATLAB: [theta, d, a, alpha]
        dh_table = matlab.double([
            [0.0, 0.0, self.L1, 0.0],
            [0.0, 0.0, self.L2, 0.0],
            [0.0, 0.0, self.L3, 0.0]
        ])

        target_pose = matlab.double([float(x), float(y), float(phi)])

        try:
            # Call inverse_kinematics.m
            angles_matlab = self.eng.inverse_kinematics(
                dh_table, target_pose, bool(elbow_up), self.x_min, self.y_min, nargout=1,stdout=self.out, stderr=self.err
            )
            theta1, theta2, theta3 = angles_matlab[0]

            # Validate joint angle limits in Python
            for i, th in enumerate((theta1, theta2, theta3), 1):
                lo, hi = self.joint_limits[i - 1]
                if not (lo - 1e-9 <= th <= hi + 1e-9):
                    raise ValueError(
                        f"Joint {i} limit exceeded: {math.degrees(th):.1f}° not in "
                        f"[{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]"
                    )

            return float(theta1), float(theta2), float(theta3)

        except matlab.engine.MatlabExecutionError as e:
            raise ValueError(str(e))
        
    def normalize_angle(self, angle):
            return (angle + math.pi) % (2 * math.pi) - math.pi
    def close(self):
        """Cleanly close the MATLAB engine session."""
        if hasattr(self, 'eng') and self.eng:
            self.eng.quit()
            print("MATLAB engine session closed.")

    def __del__(self):
        self.close()