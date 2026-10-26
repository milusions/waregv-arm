function pos = kinematics(dh_table)
    % Extract theta angles from DH parameters (Column 1)
    t1 = dh_table(1, 1);
    t2 = dh_table(2, 1);
    t3 = dh_table(3, 1);
    
    % Extract link lengths from DH parameters (Column 3)
    L1 = dh_table(1, 3);
    L2 = dh_table(2, 3);
    L3 = dh_table(3, 3);

    % Heavy trigonometric calculations in MATLAB
    phi = t1 + t2 + t3;
    x = L1 * cos(t1) + L2 * cos(t1 + t2) + L3 * cos(phi);
    y = L1 * sin(t1) + L2 * sin(t1 + t2) + L3 * sin(phi);

    pos = [x, y, phi];
end