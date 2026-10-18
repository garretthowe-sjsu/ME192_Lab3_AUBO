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

% Each row: [alpha_(i-1), a_(i-1), d_i, theta_i]
dh1 = [ alpha1,          0,  L1, th11;
        0,  a2,  L2,  th12;
        0,          a3, L3,  th13;
        alpha4, 0, L4, th14;
        alpha5, 0, L5, th15;
        0, 0, L6, th16];

dh2 = [ alpha1,          0,  L1, th21;
        0,  a2,  L2,  th22;
        0,          a3, L3,  th23;
        alpha4, 0, L4, th24;
        alpha5, 0, L5, th25;
        0, 0, L6, th26];

dh3 = [ alpha1,          0,  L1, th31;
        0,  a2,  L2,  th32;
        0,          a3, L3,  th33;
        alpha4, 0, L4, th34;
        alpha5, 0, L5, th35;
        0, 0, L6, th36];

T = jointTransforms({dh1, dh2, dh3});


% Optional: push every matrix into the base workspace as its own variable
names = fieldnames(T);
for n = 1:numel(names)
    assignin('base', names{n}, T.(names{n}));
end

%% ---------- Local functions ----------

function T = dhMod(alpha, a, d, theta)
% Modified DH: Rot_x(alpha) * Trans_x(a) * Rot_z(theta) * Trans_z(d)
    ca = cos(alpha); sa = sin(alpha);
    ct = cos(theta); st = sin(theta);
    T = [ ct,     -st,     0,   a;
          st*ca,   ct*ca, -sa, -sa*d;
          st*sa,   ct*sa,  ca,  ca*d;
          0,       0,      0,   1];
end

function T = jointTransforms(dhTables)
% dhTables: cell array of DH tables, rows = [alpha, a, d, theta]
% Returns a struct with fields T01_1, T12_1, ..., T01_2, ...
    T = struct();
    for k = 1:numel(dhTables)
        tbl = dhTables{k};
        for i = 1:size(tbl, 1)
            name = sprintf('T%d%d_%d', i-1, i, k);
            T.(name) = simplify(dhMod(tbl(i,1), tbl(i,2), tbl(i,3), tbl(i,4)));
        end
    end
end

function Tn = chainTransforms(T, k, n)
% Multiplies T01_k * T12_k * ... * T(n-1)n_k
    Tn = eye(4);
    for i = 1:n
        Tn = Tn * T.(sprintf('T%d%d_%d', i-1, i, k));
    end
    Tn = simplify(Tn);
end

names = fieldnames(T);
for n = 1:numel(names)
    fprintf('%s =\n', names{n});
    disp(T.(names{n}))
end

T06_1 = chainTransforms(T, 1, 6);
disp(round(double(T06_1)))

T06_2 = chainTransforms(T, 2, 6);
disp(round(double(T06_2)))

T06_3 = chainTransforms(T, 3, 6);
disp(round(double(T06_3)))