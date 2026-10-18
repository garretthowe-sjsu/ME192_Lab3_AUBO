clear; clc;


L1 = 0.122;
L2 = 0.1405;
L3 = -0.1215;
L4 = 0.1025;
L5 = 0.1025;
L6 = 0.094;
alpha1 = sym(pi)/2;
alpha4 = -sym(pi)/2;
alpha5 = sym(pi)/2;
a2 = 0.408;
a3 = 0.976;

th11 = deg2rad(22.92);
th12 = deg2rad(66.17);
th13 = deg2rad(-1.96);
th14 = deg2rad(34.17);
th15 = deg2rad(89.33);
th16 = deg2rad(-0.08);

th21 = deg2rad(47.2);
th22 = deg2rad(73.56);
th23 = deg2rad(-19.01);
th24 = deg2rad(23.09);
th25 = deg2rad(73.50);
th26 = deg2rad(-0.08);

th31 = deg2rad(27.17);
th32 = deg2rad(29.15);
th33 = deg2rad(-92.97);
th34 = deg2rad(16.33);
th35 = deg2rad(63.28);
th36 = deg2rad(25.44);

% Each table: [alpha_(i-1), a_(i-1), d_i, theta_offset_i]  (meters, radians)
dhTables = {
    [ alpha1,          0,  L1, th11;
        0,  a2,  L2,  th12;
        0,          a3, L3,  th13;
        alpha4, 0, L4, th14;
        alpha5, 0, L5, th15;
        0, 0, L6, th16],

    [ alpha1,          0,  L1, th21;
        0,  a2,  L2,  th22;
        0,          a3, L3,  th23;
        alpha4, 0, L4, th24;
        alpha5, 0, L5, th25;
        0, 0, L6, th26],

        [ alpha1,          0,  L1, th31;
        0,  a2,  L2,  th32;
        0,          a3, L3,  th33;
        alpha4, 0, L4, th34;
        alpha5, 0, L5, th35;
        0, 0, L6, th36];
};

for k = 1:numel(dhTables)
    writeUrdf(dhTables{k}, sprintf('robot_%d', k));
end

function writeUrdf(dh, robotName)
    n = size(dh, 1);
    lim = repmat([-pi pi 10 2], n, 1);   % [lower upper effort velocity]
    fid = fopen([robotName '.urdf'], 'w');
    fprintf(fid, '<?xml version="1.0"?>\n<robot name="%s">\n', robotName);
    for i = 0:n
        fprintf(fid, '  <link name="link%d"/>\n', i);
    end
    for i = 1:n
        T = dhMod(dh(i,1), dh(i,2), dh(i,3), dh(i,4));
        R = T(1:3,1:3);  p = T(1:3,4);
        roll  = atan2(R(3,2), R(3,3));
        pitch = -asin(max(-1, min(1, R(3,1))));
        yaw   = atan2(R(2,1), R(1,1));
        fprintf(fid, '  <joint name="joint%d" type="revolute">\n', i);
        fprintf(fid, '    <parent link="link%d"/>\n    <child link="link%d"/>\n', i-1, i);
        fprintf(fid, '    <origin xyz="%.6f %.6f %.6f" rpy="%.6f %.6f %.6f"/>\n', p, roll, pitch, yaw);
        fprintf(fid, '    <axis xyz="0 0 1"/>\n');
        fprintf(fid, '    <limit lower="%.4f" upper="%.4f" effort="%g" velocity="%g"/>\n', lim(i,:));
        fprintf(fid, '  </joint>\n');
    end
    fprintf(fid, '</robot>\n');
    fclose(fid);
end

function T = dhMod(alpha, a, d, theta)
    ca = cos(alpha); sa = sin(alpha); ct = cos(theta); st = sin(theta);
    T = [ ct, -st, 0, a; st*ca, ct*ca, -sa, -sa*d; st*sa, ct*sa, ca, ca*d; 0 0 0 1];
end