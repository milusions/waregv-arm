import math

class Planar3DOFArm:
    def __init__(self, l1, l2, l3, x_min=0.0, y_min=-50.0, joint_limits=None):
        # joint_limits: [(min, max), (min, max), (min, max)] in radians for theta1..3.
        # Default is +/-120 deg; the app replaces it with the limits read from the servos.
        lim = math.radians(120)
        self.joint_limits = joint_limits or [(-lim, lim)] * 3
        self.L1 = l1
        self.L2 = l2
        self.L3 = l3
        self.x_min = x_min
        self.y_min = y_min

    def forward_kinematics(self, theta1, theta2, theta3):
        phi = theta1 + theta2 + theta3
        
        x = (self.L1 * math.cos(theta1) + 
             self.L2 * math.cos(theta1 + theta2) + 
             self.L3 * math.cos(phi))
             
        y = (self.L1 * math.sin(theta1) + 
             self.L2 * math.sin(theta1 + theta2) + 
             self.L3 * math.sin(phi))
             
        return x, y, phi

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def inverse_kinematics(self, x, y, phi, elbow_up=True):
        if x < self.x_min:
            raise ValueError(f"Target x < {self.x_min} is not allowed.")
        if y < self.y_min:
            raise ValueError(f"Target y < {self.y_min} is not allowed.")

        # Step 1: Find the wrist center
        xw = x - self.L3 * math.cos(phi)
        yw = y - self.L3 * math.sin(phi)
        
        # Step 2: Solve for theta2
        D = (xw**2 + yw**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        
        if abs(D) > 1:
            raise ValueError("Target is out of reach.")
            
        sin_theta2 = math.sqrt(1 - D**2)
        if not elbow_up:
            sin_theta2 = sin_theta2
            
        theta2 = math.atan2(sin_theta2, D)
        
        # Step 3: Solve for theta1
        theta1 = math.atan2(yw, xw) - math.atan2(self.L2 * math.sin(theta2), 
                                                 self.L1 + self.L2 * math.cos(theta2))
                                                 
        # Step 4: Solve for theta3
        theta3 = phi - theta1 - theta2
        
        theta1 = self.normalize_angle(theta1)
        theta2 = self.normalize_angle(theta2)
        theta3 = self.normalize_angle(theta3)
        
        for i, th in enumerate((theta1, theta2, theta3), 1):
            lo, hi = self.joint_limits[i - 1]
            if not (lo - 1e-9 <= th <= hi + 1e-9):
                raise ValueError(f"Joint {i} limit exceeded: {math.degrees(th):.1f}° not in "
                                 f"[{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]")
        
        x1 = self.L1 * math.cos(theta1)
        x2 = x1 + self.L2 * math.cos(theta1 + theta2)
        x3 = x2 + self.L3 * math.cos(theta1 + theta2 + theta3)
        
        y1 = self.L1 * math.sin(theta1)
        y2 = y1 + self.L2 * math.sin(theta1 + theta2)
        y3 = y2 + self.L3 * math.sin(theta1 + theta2 + theta3)
        
        if x1 < self.x_min - 1e-6 or x2 < self.x_min - 1e-6 or x3 < self.x_min - 1e-6:
             raise ValueError(f"Constraint violated: Arm links entered x < {self.x_min} region.")
             
        if y1 < self.y_min - 1e-6 or y2 < self.y_min - 1e-6 or y3 < self.y_min - 1e-6:
             raise ValueError(f"Constraint violated: Arm links entered y < {self.y_min} region.")
             
        return theta1, theta2, theta3