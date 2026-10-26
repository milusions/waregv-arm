function joint_angles = inverse_kinematics(dh_table, target_pose, elbow_up, x_min, y_min)
    if nargin < 3, elbow_up = true; end
    if nargin < 4, x_min = 0.0; end
    if nargin < 5, y_min = -50.0; end

    L1 = dh_table(1, 3);
    L2 = dh_table(2, 3);
    L3 = dh_table(3, 3);

    x = target_pose(1);
    y = target_pose(2);
    phi = target_pose(3);

    % Bound checks
    if x < x_min
        error('Target x < %f is not allowed.', x_min);
    end
    if y < y_min
        error('Target y < %f is not allowed.', y_min);
    end

    % Step 1: Wrist Center Calculation
    xw = x - L3 * cos(phi);
    yw = y - L3 * sin(phi);

    % Step 2: Solve for theta2
    D = (xw^2 + yw^2 - L1^2 - L2^2) / (2 * L1 * L2);
    if abs(D) > 1
        error('Target is out of reach.');
    end

    sin_theta2 = sqrt(1 - D^2);
    if ~elbow_up
        sin_theta2 = -sin_theta2;
    end
    theta2 = atan2(sin_theta2, D);

    % Step 3: Solve for theta1
    theta1 = atan2(yw, xw) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));

    % Step 4: Solve for theta3
    theta3 = phi - theta1 - theta2;

    % Angle Normalization (-pi to pi)
    theta1 = atan2(sin(theta1), cos(theta1));
    theta2 = atan2(sin(theta2), cos(theta2));
    theta3 = atan2(sin(theta3), cos(theta3));

    % Link position validation checks in MATLAB
    x1 = L1 * cos(theta1);
    x2 = x1 + L2 * cos(theta1 + theta2);
    x3 = x2 + L3 * cos(theta1 + theta2 + theta3);

    y1 = L1 * sin(theta1);
    y2 = y1 + L2 * sin(theta1 + theta2);
    y3 = y2 + L3 * sin(theta1 + theta2 + theta3);

    if x1 < x_min - 1e-6 || x2 < x_min - 1e-6 || x3 < x_min - 1e-6
        error('Constraint violated: Arm links entered x < %f region.', x_min);
    end

    if y1 < y_min - 1e-6 || y2 < y_min - 1e-6 || y3 < y_min - 1e-6
        error('Constraint violated: Arm links entered y < %f region.', y_min);
    end

    joint_angles = [theta1, theta2, theta3];
end