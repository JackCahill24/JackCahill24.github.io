clear; clc;

%% MEEG671 Final Project - John Cahill

%% Robot limits (LBR iiwa 7 R800)

q_min_deg = [-170; -120; -170; -120; -170; -120; -175];
q_max_deg = [ 170;  120;  170;  120;  170;  120;  175];

qdot_max_deg = [98; 98; 100; 130; 140; 180; 180];

q_min = deg2rad(q_min_deg);
q_max = deg2rad(q_max_deg);
q_mid = (q_min + q_max)/2;
qdot_max = deg2rad(qdot_max_deg);

%% Trajectory planning parameters

dt = 0.005;      % Sampling rate [s]
T1 = 8.0;        % Approach phase duration [s]
T2 = 3.0;        % Each linear phase duration [s]
Thold = 3.0;     % Holding phase duration [s]

%% Known transformations and configurations

% Transformation matrix from end-effector frame to stamp frame
Tes = [0 -1 0 0;
       -1 0 0 0.0455;
       0 0 -1 0.0600;
       0 0 0 1];

% Transformation matrix from end-effector frame to camera frame
Tec = [ 0   1   0   0;
       -1   0   0  -0.0662;
        0   0   1   0.0431;
        0   0   0   1];

% Transformation matrix from camera frame to ArUco marker frame
px = -0.019795009034628;
py = -0.033101516609441;
pz =  0.620680522263168;

roll  = deg2rad(178.5474772244808);
pitch = deg2rad(-4.934483303290795);
yaw   = deg2rad(-37.688034287215352);
Rca = eul2rotm([yaw pitch roll], 'ZYX');

Tca = [Rca, [px; py; pz];
       0 0 0 1];

% Known robot configuration when image was captured
qc_deg = [55.38; 9.28; -138.54; 84.24; -6.20; -82.98; 141.00];
qc = deg2rad(qc_deg);

% Initial robot configuration
q1_deg = [58.2686; 75.3224; 11.7968; 45.9029; -22.1081; -31.2831; -42.3712];
q1 = deg2rad(q1_deg);

% Compute T0e at qc
[T0e_qc, ~, ~] = fk_T0e_KUKA(qc);

%% ArUco marker frame target transforms

Tap1 = [1 0 0  0.033975;
        0 1 0 -0.176975;
        0 0 1  0;
        0 0 0  1];

Tap2 = [1 0 0  0.033975;
        0 1 0 -0.103975;
        0 0 1  0;
        0 0 0  1];

Tap3 = [1 0 0  0.103975;
        0 1 0 -0.176975;
        0 0 1  0;
        0 0 0  1];

Tap4 = [1 0 0  0.103975;
        0 1 0 -0.103975;
        0 0 1  0;
        0 0 0  1];

%% Approach phase: Compute qP1 using inverse Jacobian method with secondary objectives

T0P1 = T0e_qc*Tec*Tca*Tap1;
xd_P1 = pose_from_transform(T0P1);

qP1 = solve_IK_stamp_KUKA_secobj( ...
    xd_P1, q1, Tes, q_min, q_max, q_mid);

qP1 = mod(qP1 + pi, 2*pi) - pi;

[T0e_P1, ~, ~] = fk_T0e_KUKA(qP1);

disp('Configuration qP1 =')
disp(qP1)

disp('Transformation matrix T0e(qP1) =')
disp(T0e_P1)

%% Approach phase: Compute trajectory (q1 -> qP1) with 3rd order polynomial

t_approach = (0:dt:(T1-dt))';

N_approach = length(t_approach);
n_joints = length(q1);

q_approach = zeros(N_approach, n_joints);
qdot_approach = zeros(N_approach, n_joints);
qddot_approach = zeros(N_approach, n_joints);

for j = 1:n_joints
    a0 = q1(j);
    a1 = 0;
    a2 = -3*(q1(j) - qP1(j))/T1^2;
    a3 =  2*(q1(j) - qP1(j))/T1^3;

    q_approach(:,j)     = a3*t_approach.^3 + a2*t_approach.^2 + a1*t_approach + a0;
    qdot_approach(:,j)  = 3*a3*t_approach.^2 + 2*a2*t_approach + a1;
    qddot_approach(:,j) = 6*a3*t_approach + 2*a2;
end

pos_ok_approach = all(all(q_approach >= q_min' & q_approach <= q_max'));
vel_ok_approach = all(all(abs(qdot_approach) <= qdot_max'));

disp('Approach phase joint position limits satisfied?')
disp(pos_ok_approach)

disp('Approach phase joint velocity limits satisfied?')
disp(vel_ok_approach)

disp('Approach phase trajectory size =')
disp(size(q_approach))

q_traj = q_approach;

%% Linear phase 1: Compute trajectory (P1 -> P2)
% Use 3rd order polynomial on changing Cartesian coordinate y in ArUco frame

t_linear1 = (0:dt:(T2-dt))';
N_linear1 = length(t_linear1);
q_linear1 = zeros(N_linear1, n_joints);

q_guess = qP1;

xA = Tap1(1,4);
yA_start = Tap1(2,4);
yA_end   = Tap2(2,4);
zA = Tap1(3,4);

[a0_y1, a1_y1, a2_y1, a3_y1] = cubic_coefficients(yA_start, yA_end, 0, 0, T2);

for k = 1:N_linear1

    tk = t_linear1(k);
    yA_k = a3_y1*tk^3 + a2_y1*tk^2 + a1_y1*tk + a0_y1;

    Tap_k = [1 0 0  xA;
             0 1 0  yA_k;
             0 0 1  zA;
             0 0 0  1];

    T0d_k = T0e_qc*Tec*Tca*Tap_k;
    xd_k = pose_from_transform(T0d_k);

    q_k = solve_IK_stamp_KUKA_secobj( ...
        xd_k, q_guess, Tes, q_min, q_max, q_mid);

    q_k = mod(q_k + pi, 2*pi) - pi;

    q_linear1(k,:) = q_k';
    q_guess = q_k;
end

qdot_linear1 = zeros(size(q_linear1));
qdot_linear1(2:end,:) = diff(q_linear1)/dt;
qdot_linear1(1,:) = qdot_linear1(2,:);

pos_ok_linear1 = all(all(q_linear1 >= q_min' & q_linear1 <= q_max'));
vel_ok_linear1 = all(all(abs(qdot_linear1) <= qdot_max'));

disp('Linear phase 1 joint position limits satisfied?')
disp(pos_ok_linear1)

disp('Linear phase 1 joint velocity limits satisfied?')
disp(vel_ok_linear1)

disp('Linear phase 1 trajectory size =')
disp(size(q_linear1))

q_traj = [q_traj; q_linear1];

%% Linear phase 2: Compute trajectory (P2 -> P3)
% Use 3rd order polynomial on changing Cartesian coordinates x and y in ArUco frame

t_linear2 = (0:dt:(T2-dt))';
N_linear2 = length(t_linear2);
q_linear2 = zeros(N_linear2, n_joints);

q_guess = q_linear1(end,:)';

xA_start = Tap2(1,4);
xA_end   = Tap3(1,4);
yA_start = Tap2(2,4);
yA_end   = Tap3(2,4);
zA = Tap2(3,4);

[a0_x2, a1_x2, a2_x2, a3_x2] = cubic_coefficients(xA_start, xA_end, 0, 0, T2);
[a0_y2, a1_y2, a2_y2, a3_y2] = cubic_coefficients(yA_start, yA_end, 0, 0, T2);

for k = 1:N_linear2

    tk = t_linear2(k);
    xA_k = a3_x2*tk^3 + a2_x2*tk^2 + a1_x2*tk + a0_x2;
    yA_k = a3_y2*tk^3 + a2_y2*tk^2 + a1_y2*tk + a0_y2;

    Tap_k = [1 0 0  xA_k;
             0 1 0  yA_k;
             0 0 1  zA;
             0 0 0  1];

    T0d_k = T0e_qc*Tec*Tca*Tap_k;
    xd_k = pose_from_transform(T0d_k);

    q_k = solve_IK_stamp_KUKA_secobj( ...
        xd_k, q_guess, Tes, q_min, q_max, q_mid);

    q_k = mod(q_k + pi, 2*pi) - pi;

    q_linear2(k,:) = q_k';
    q_guess = q_k;
end

qdot_linear2 = zeros(size(q_linear2));
qdot_linear2(2:end,:) = diff(q_linear2)/dt;
qdot_linear2(1,:) = qdot_linear2(2,:);

pos_ok_linear2 = all(all(q_linear2 >= q_min' & q_linear2 <= q_max'));
vel_ok_linear2 = all(all(abs(qdot_linear2) <= qdot_max'));

disp('Linear phase 2 joint position limits satisfied?')
disp(pos_ok_linear2)

disp('Linear phase 2 joint velocity limits satisfied?')
disp(vel_ok_linear2)

disp('Linear phase 2 trajectory size =')
disp(size(q_linear2))

q_traj = [q_traj; q_linear2];

%% Linear phase 3: Compute trajectory (P3 -> P4)
% Use 3rd order polynomial on changing Cartesian coordinate y in ArUco frame

t_linear3 = (0:dt:(T2-dt))';
N_linear3 = length(t_linear3);
q_linear3 = zeros(N_linear3, n_joints);

q_guess = q_linear2(end,:)';

xA = Tap3(1,4);
yA_start = Tap3(2,4);
yA_end   = Tap4(2,4);
zA = Tap3(3,4);

[a0_y3, a1_y3, a2_y3, a3_y3] = cubic_coefficients(yA_start, yA_end, 0, 0, T2);

for k = 1:N_linear3

    tk = t_linear3(k);
    yA_k = a3_y3*tk^3 + a2_y3*tk^2 + a1_y3*tk + a0_y3;

    Tap_k = [1 0 0  xA;
             0 1 0  yA_k;
             0 0 1  zA;
             0 0 0  1];

    T0d_k = T0e_qc*Tec*Tca*Tap_k;
    xd_k = pose_from_transform(T0d_k);

    q_k = solve_IK_stamp_KUKA_secobj( ...
        xd_k, q_guess, Tes, q_min, q_max, q_mid);

    q_k = mod(q_k + pi, 2*pi) - pi;

    q_linear3(k,:) = q_k';
    q_guess = q_k;
end

qdot_linear3 = zeros(size(q_linear3));
qdot_linear3(2:end,:) = diff(q_linear3)/dt;
qdot_linear3(1,:) = qdot_linear3(2,:);

pos_ok_linear3 = all(all(q_linear3 >= q_min' & q_linear3 <= q_max'));
vel_ok_linear3 = all(all(abs(qdot_linear3) <= qdot_max'));

disp('Linear phase 3 joint position limits satisfied?')
disp(pos_ok_linear3)

disp('Linear phase 3 joint velocity limits satisfied?')
disp(vel_ok_linear3)

disp('Linear phase 3 trajectory size =')
disp(size(q_linear3))

q_traj = [q_traj; q_linear3];

%% Final holding phase: Hold final configuration at P4

N_hold = Thold/dt;
q_hold = repmat(q_linear3(end,:), N_hold, 1);

qdot_hold = zeros(size(q_hold));

pos_ok_hold = all(all(q_hold >= q_min' & q_hold <= q_max'));
vel_ok_hold = all(all(abs(qdot_hold) <= qdot_max'));

disp('Final holding phase joint position limits satisfied?')
disp(pos_ok_hold)

disp('Final holding phase joint velocity limits satisfied?')
disp(vel_ok_hold)

disp('Final holding phase trajectory size =')
disp(size(q_hold))

q_traj = [q_traj; q_hold];

%% Full trajectory joint position and velocity limit check

% Compute full-trajectory joint velocities using finite differences
qdot_traj = zeros(size(q_traj));
qdot_traj(2:end,:) = diff(q_traj)/dt;
qdot_traj(1,:) = qdot_traj(2,:);

% Check position limits
pos_low_violation  = q_traj < q_min';
pos_high_violation = q_traj > q_max';
pos_violation = pos_low_violation | pos_high_violation;

% Check velocity limits
vel_violation = abs(qdot_traj) > qdot_max';

% Overall pass/fail checks
pos_ok_full = ~any(pos_violation, 'all');
vel_ok_full = ~any(vel_violation, 'all');

disp('Full trajectory joint position limits satisfied?')
disp(pos_ok_full)

disp('Full trajectory joint velocity limits satisfied?')
disp(vel_ok_full)

% Report detailed position violations, if any
if ~pos_ok_full
    disp('Position limit violations found:')

    [row_pos, joint_pos] = find(pos_violation);

    for i = 1:length(row_pos)
        r = row_pos(i);
        j = joint_pos(i);

        fprintf(['Row %d, Joint %d: q = %.6f rad (%.3f deg), ', ...
                 'limits = [%.6f, %.6f] rad ([%.3f, %.3f] deg)\n'], ...
                 r, j, ...
                 q_traj(r,j), rad2deg(q_traj(r,j)), ...
                 q_min(j), q_max(j), ...
                 rad2deg(q_min(j)), rad2deg(q_max(j)));
    end
end

% Report detailed velocity violations, if any
if ~vel_ok_full
    disp('Velocity limit violations found:')

    [row_vel, joint_vel] = find(vel_violation);

    for i = 1:length(row_vel)
        r = row_vel(i);
        j = joint_vel(i);

        fprintf(['Row %d -> %d, Joint %d: qdot = %.6f rad/s (%.3f deg/s), ', ...
                 'limit = %.6f rad/s (%.3f deg/s)\n'], ...
                 r-1, r, j, ...
                 qdot_traj(r,j), rad2deg(qdot_traj(r,j)), ...
                 qdot_max(j), rad2deg(qdot_max(j)));
    end
end

%% Final trajectory size check and export final trajectory to .txt file

disp('Final full trajectory size =')
disp(size(q_traj))

fid = fopen('Cahill_John.txt', 'w');

for i = 1:size(q_traj,1)
    fprintf(fid, '%.8f %.8f %.8f %.8f %.8f %.8f %.8f\n', q_traj(i,:));
end

fclose(fid);

disp('Trajectory exported to Cahill_John.txt')

%% Forward kinematics function for KUKA
function [T0e, p_all, z_all] = fk_T0e_KUKA(q)

    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4);
    q5 = q(5); q6 = q(6); q7 = q(7);

    d1 = 0.340; d2 = 0.000; d3 = 0.400; d4 = 0.000;
    d5 = 0.400; d6 = 0.000; d7 = 0.126;

    a1 = 0.000; a2 = 0.000; a3 = 0.000; a4 = 0.000;
    a5 = 0.000; a6 = 0.000; a7 = 0.000;

    alpha1 = -pi/2; alpha2 =  pi/2; alpha3 =  pi/2; alpha4 = -pi/2;
    alpha5 = -pi/2; alpha6 =  pi/2; alpha7 =  0;

    A01 = dh_transform(q1, d1, a1, alpha1);
    A12 = dh_transform(q2, d2, a2, alpha2);
    A23 = dh_transform(q3, d3, a3, alpha3);
    A34 = dh_transform(q4, d4, a4, alpha4);
    A45 = dh_transform(q5, d5, a5, alpha5);
    A56 = dh_transform(q6, d6, a6, alpha6);
    A6e = dh_transform(q7, d7, a7, alpha7);

    T00 = eye(4);
    T01 = A01;
    T02 = T01*A12;
    T03 = T02*A23;
    T04 = T03*A34;
    T05 = T04*A45;
    T06 = T05*A56;
    T0e = T06*A6e;

    p_all = [T00(1:3,4), T01(1:3,4), T02(1:3,4), T03(1:3,4), ...
             T04(1:3,4), T05(1:3,4), T06(1:3,4), T0e(1:3,4)];

    z_all = [T00(1:3,3), T01(1:3,3), T02(1:3,3), T03(1:3,3), ...
             T04(1:3,3), T05(1:3,3), T06(1:3,3)];
end

%% Forward kinematics function for KUKA with stamp
function xs = fwd_kin_stamp_KUKA(q, Tes)

    [T0e, ~, ~] = fk_T0e_KUKA(q);
    T0s = T0e*Tes;
    xs = pose_from_transform(T0s);

end

%% Computes the analytical Jacobian for KUKA with stamp
function Ja = an_Jac_stamp_KUKA(q, Tes)

    [T0e, p_all, z_all] = fk_T0e_KUKA(q);
    T0s = T0e*Tes;

    ps = T0s(1:3,4);

    Jp1 = cross(z_all(:,1), ps - p_all(:,1));
    Jp2 = cross(z_all(:,2), ps - p_all(:,2));
    Jp3 = cross(z_all(:,3), ps - p_all(:,3));
    Jp4 = cross(z_all(:,4), ps - p_all(:,4));
    Jp5 = cross(z_all(:,5), ps - p_all(:,5));
    Jp6 = cross(z_all(:,6), ps - p_all(:,6));
    Jp7 = cross(z_all(:,7), ps - p_all(:,7));

    Jo1 = z_all(:,1);
    Jo2 = z_all(:,2);
    Jo3 = z_all(:,3);
    Jo4 = z_all(:,4);
    Jo5 = z_all(:,5);
    Jo6 = z_all(:,6);
    Jo7 = z_all(:,7);

    Jg = [Jp1 Jp2 Jp3 Jp4 Jp5 Jp6 Jp7;
          Jo1 Jo2 Jo3 Jo4 Jo5 Jo6 Jo7];

    Rs = T0s(1:3,1:3);

    phi   = atan2(Rs(2,3), Rs(1,3));
    theta = atan2(sqrt(Rs(1,3)^2 + Rs(2,3)^2), Rs(3,3));

    Tphi = [0      -sin(phi)     cos(phi)*sin(theta);
            0       cos(phi)     sin(phi)*sin(theta);
            1       0            cos(theta)];

    TA = [eye(3)      zeros(3,3);
          zeros(3,3)  Tphi];

    Ja = TA \ Jg;

end

%% Extract pose vector from a homogeneous transformation matrix
function x = pose_from_transform(T)

    R = T(1:3,1:3);
    p = T(1:3,4);

    phi   = atan2(R(2,3), R(1,3));
    theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    psi   = atan2(R(3,2), -R(3,1));

    x = [p; phi; theta; psi];

end

%% Solves inverse kinematics for a desired stamp pose using Jacobian inverse method with null-space secondary objectives
function q_sol = solve_IK_stamp_KUKA_secobj( ...
    xd, q_init, Tes, q_min, q_max, q_mid)

    % Solver settings
    K = eye(6);
    tol = 1e-4;
    max_iter = 1000;
    dt = 0.01;

    % Universal secondary objective gains
    k_sing  = 1;
    k_lim   = 1000;
    k_table = 1;

    q = q_init;
    n = length(q);

    for i = 1:max_iter

        xs = fwd_kin_stamp_KUKA(q, Tes);
        Ja = an_Jac_stamp_KUKA(q, Tes);

        e = xd - xs;

        if max(abs(e)) < tol
            break
        end

        Jpinv = pinv(Ja);
        qdot_primary = Jpinv*K*e;

        grad_sing  = grad_manipulability_objective(q, Tes);
        grad_lim   = grad_joint_limit_objective(q, q_min, q_max, q_mid);
        grad_table = grad_table_objective(q, Tes);

        qdot0 = k_sing*grad_sing + k_lim*grad_lim + k_table*grad_table;

        N = eye(n) - Jpinv*Ja;
        qdot = qdot_primary + N*qdot0;

        q = q + qdot*dt;
    end

    q_sol = q;

end

%% Gradient of manipulability objective
function grad_w = grad_manipulability_objective(q, Tes)

    eps_fd = 1e-6;
    n = length(q);
    grad_w = zeros(n,1);

    Ja0 = an_Jac_stamp_KUKA(q, Tes);
    JJt0 = Ja0*Ja0';
    w0 = sqrt(max(det(JJt0), 0));

    for i = 1:n
        q_pert = q;
        q_pert(i) = q_pert(i) + eps_fd;

        Ja1 = an_Jac_stamp_KUKA(q_pert, Tes);
        JJt1 = Ja1*Ja1';
        w1 = sqrt(max(det(JJt1), 0));

        grad_w(i) = (w1 - w0)/eps_fd;
    end

end

%% Gradient of joint-limit objective
function grad_w = grad_joint_limit_objective(q, q_min, q_max, q_mid)

    n = length(q);
    grad_w = zeros(n,1);

    for i = 1:n
        denom = (q_max(i) - q_min(i))^2;
        grad_w(i) = -(1/n) * (q(i) - q_mid(i)) / denom;
    end

end

%% Gradient of table-clearance objective
function grad_w = grad_table_objective(q, Tes)

    eps_fd = 1e-6;
    n = length(q);
    grad_w = zeros(n,1);

    w0 = table_distance_objective(q, Tes);

    for i = 1:n
        q_pert = q;
        q_pert(i) = q_pert(i) + eps_fd;

        w1 = table_distance_objective(q_pert, Tes);

        grad_w(i) = (w1 - w0)/eps_fd;
    end

end

%% Table distance objective
function w = table_distance_objective(q, Tes)

    [T0e, p_all, ~] = fk_T0e_KUKA(q);
    T0s = T0e*Tes;
    ps = T0s(1:3,4);

    monitored_points = [p_all, ps];

    num_points = size(monitored_points, 2);
    distances = zeros(num_points, 1);

    for k = 1:num_points
        distances(k) = point_to_table_distance(monitored_points(:,k));
    end

    w = min(distances);

end

%% Distance from a point to the table half-box obstacle
function d = point_to_table_distance(p)

    x = p(1);
    y = p(2);
    z = p(3);

    xc = min(x, 0.330);
    yc = min(max(y, -0.330), 0.330);
    zc = min(z, 0);

    p_closest = [xc; yc; zc];
    d = norm(p - p_closest);

end

%% Standard DH transformation matrix
function A = dh_transform(theta, d, a, alpha)

    A = [ cos(theta)   -sin(theta)*cos(alpha)    sin(theta)*sin(alpha)    a*cos(theta);
          sin(theta)    cos(theta)*cos(alpha)   -cos(theta)*sin(alpha)    a*sin(theta);
          0             sin(alpha)               cos(alpha)               d;
          0             0                        0                        1 ];

end

%% 3rd-order polynomial coefficients for scalar coordinate interpolation
function [a0, a1, a2, a3] = cubic_coefficients(pi, pf, pdi, pdf, tf)

    a0 = pi;
    a1 = pdi;
    a2 = (-3*(pi - pf) - (2*pdi + pdf)*tf) / tf^2;
    a3 = ( 2*(pi - pf) + (pdi + pdf)*tf) / tf^3;

end