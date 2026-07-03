clear; clc; close all;

%% Standalone fast simulation from trajectory .txt file

filename = 'Cahill_John.txt';
dt = 0.005;

q_traj = readmatrix(filename);

if size(q_traj,2) ~= 7
    error('Trajectory file must contain 7 columns.');
end

N = size(q_traj,1);

%% Tool transform
Tes = [0 -1 0 0;
       -1 0 0 0.0455;
       0 0 -1 0.0600;
       0 0 0 1];

%% Precompute robot joint points and stamp path
p_joint = zeros(3,8,N);   % 8 frame origins: base through end effector
p_stamp = zeros(3,N);

for k = 1:N
    qk = q_traj(k,:)';

    [T0e_k, p_all_k, ~] = fk_T0e_KUKA_vis(qk);
    T0s_k = T0e_k * Tes;

    p_joint(:,:,k) = p_all_k;
    p_stamp(:,k) = T0s_k(1:3,4);
end

%% Create figure
fig = figure('Name','Fast KUKA Simulation From TXT');
ax = axes(fig);
hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');

xlabel(ax,'X [m]');
ylabel(ax,'Y [m]');
zlabel(ax,'Z [m]');
title(ax,'Real-Time KUKA Trajectory Simulation');

xlim(ax,[-0.6 0.8]);
ylim(ax,[-0.6 0.8]);
zlim(ax,[-0.2 1.0]);
view(ax,45,25);

%% Draw static objects once

% Table top
x_table = [-0.5, 0.33, 0.33, -0.5];
y_table = [-0.33, -0.33, 0.33, 0.33];
z_table = [0, 0, 0, 0];

patch(ax, x_table, y_table, z_table, [0.8 0.8 0.8], ...
    'FaceAlpha', 0.4, 'EdgeColor', 'k');

% Full stamp path
plot3(ax, p_stamp(1,:), p_stamp(2,:), p_stamp(3,:), ...
    'k--', 'LineWidth', 1.0);

% Base origin
plot3(ax, 0, 0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);

%% Create animated objects once

% Robot links/joints
robot_line = plot3(ax, nan, nan, nan, 'b-o', ...
    'LineWidth', 2, 'MarkerSize', 5, 'MarkerFaceColor', 'b');

% Current stamp point
stamp_point = plot3(ax, nan, nan, nan, 'm.', 'MarkerSize', 20);

% Stamp trace up to current point
stamp_trace = plot3(ax, nan, nan, nan, 'm-', 'LineWidth', 1.5);

% Time display
time_text = text(ax, -0.55, 0.70, 0.90, 't = 0.000 s', ...
    'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', 'w');

%% Real-time looping animation
while ishandle(fig)

    for k = 1:N

        if ~ishandle(fig)
            break
        end

        pk = p_joint(:,:,k);

        % Update robot
        set(robot_line, ...
            'XData', pk(1,:), ...
            'YData', pk(2,:), ...
            'ZData', pk(3,:));

        % Update current stamp point
        set(stamp_point, ...
            'XData', p_stamp(1,k), ...
            'YData', p_stamp(2,k), ...
            'ZData', p_stamp(3,k));

        % Update trace
        set(stamp_trace, ...
            'XData', p_stamp(1,1:k), ...
            'YData', p_stamp(2,1:k), ...
            'ZData', p_stamp(3,1:k));

        % Update displayed trajectory time
        current_time = (k-1) * dt;
        set(time_text, 'String', sprintf('t = %.3f s', current_time));

        drawnow limitrate;
        pause(dt);
    end

    pause(0.5);
end

%% Local functions
function [T0e, p_all, z_all] = fk_T0e_KUKA_vis(q)

    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4);
    q5 = q(5); q6 = q(6); q7 = q(7);

    d1 = 0.340; d2 = 0.000; d3 = 0.400; d4 = 0.000;
    d5 = 0.400; d6 = 0.000; d7 = 0.126;

    a1 = 0.000; a2 = 0.000; a3 = 0.000; a4 = 0.000;
    a5 = 0.000; a6 = 0.000; a7 = 0.000;

    alpha1 = -pi/2;
    alpha2 =  pi/2;
    alpha3 =  pi/2;
    alpha4 = -pi/2;
    alpha5 = -pi/2;
    alpha6 =  pi/2;
    alpha7 =  0;

    A01 = dh_transform_vis(q1, d1, a1, alpha1);
    A12 = dh_transform_vis(q2, d2, a2, alpha2);
    A23 = dh_transform_vis(q3, d3, a3, alpha3);
    A34 = dh_transform_vis(q4, d4, a4, alpha4);
    A45 = dh_transform_vis(q5, d5, a5, alpha5);
    A56 = dh_transform_vis(q6, d6, a6, alpha6);
    A6e = dh_transform_vis(q7, d7, a7, alpha7);

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

function A = dh_transform_vis(theta, d, a, alpha)

    A = [ cos(theta)   -sin(theta)*cos(alpha)    sin(theta)*sin(alpha)    a*cos(theta);
          sin(theta)    cos(theta)*cos(alpha)   -cos(theta)*sin(alpha)    a*sin(theta);
          0             sin(alpha)               cos(alpha)               d;
          0             0                        0                        1 ];
end