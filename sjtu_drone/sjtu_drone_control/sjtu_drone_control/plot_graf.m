% plot_trajectory_errors.m
% Uso:
%   1) pon este .m en la misma carpeta que trajectory_errors.csv
%   2) octave plot_trajectory_errors.m
%
% Columnas esperadas (según tu CSV):
% t_sec, wp_id, target_x, target_y, target_z, pose_x, pose_y, pose_z, err_x, err_y, err_z, err_norm

clear; close all; clc;

fname = "trajectory_errors.csv";

% Lee CSV saltando la cabecera (1ª fila)
data = dlmread(fname, ",", 1, 0);

t        = data(:,1);  t = t - t(1);        % tiempo (re-referenciado a 0)
wp_id    = data(:,2);

target_x = data(:,3);  target_y = data(:,4);  target_z = data(:,5);
pose_x   = data(:,6);  pose_y   = data(:,7);  pose_z   = data(:,8);

err_x    = data(:,9);  err_y    = data(:,10); err_z    = data(:,11);
err_norm = data(:,12);

% -------- Plot 1: norma del error vs tiempo
figure(1); clf;
plot(t, err_norm, "LineWidth", 1.5);
grid on;
xlabel("t [s]");
ylabel("||e|| [m]");
title("Trajectory tracking error norm");

% -------- Plot 2: componentes del error vs tiempo
figure(2); clf;
plot(t, err_x, "LineWidth", 1.2); hold on;
plot(t, err_y, "LineWidth", 1.2);
plot(t, err_z, "LineWidth", 1.2);
grid on;
xlabel("t [s]");
ylabel("e_x, e_y, e_z [m]");
legend("e_x", "e_y", "e_z", "location", "best");
title("Tracking error components");

% -------- Plot 3: trayectoria 3D (pose vs target)
figure(3); clf;
plot3(pose_x, pose_y, pose_z, "LineWidth", 1.5); hold on;
plot3(target_x, target_y, target_z, "--", "LineWidth", 1.5);
grid on; axis equal;
xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]");
legend("pose", "target", "location", "best");
title("3D trajectory: pose vs target");

% (Opcional) Guardar a PNG
figure(1); print("-dpng", "err_norm_vs_time.png");
figure(2); print("-dpng", "err_xyz_vs_time.png");
figure(3); print("-dpng", "traj_3d_pose_vs_target.png");

disp("Listo: generadas figuras y PNGs en la carpeta actual.");

