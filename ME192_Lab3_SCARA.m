clear; clc;

L1 = 0.3937;
L3 = .0374;


alpha3 = sym(pi);

a2 = 0.3556;
a3 = 0.2921;

th11 = deg2rad(200);
th12 = deg2rad(190);
th13 = deg2rad(130);
th14 = deg2rad(40.97);

th21 = deg2rad(-44.33);
th22 = deg2rad(108.104);
th23 = deg2rad(20.03);
th24 = deg2rad(-143.65);

th31 = deg2rad(116.86);
th32 = deg2rad(211.53);
th33 = deg2rad(101.19);
th34 = deg2rad(40.97);

% Each table: [alpha_(i-1), a_(i-1), d_i, theta_offset_i]  (meters, radians)
    dh1 = [ 0,          0,  L1, th11;
        0,  a2,  0,  th12;
        alpha3,          a3, L3,  th13;
        0,  0,  0,  th14;
        ];

    dh2 = [ 0,          0,  L1, th21;
        0,  a2,  0,  th22;
        alpha3,          a3, L3,  th23;
        0,  0,  0,  th24;
        ];

        dh3 =[ 0,          0,  L1, th31;
        0,  a2,  0,  th32;
        alpha3,          a3, L3,  th33;
        0,  0,  0,  th34;
];


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