% plot_graf.m
% Espera el CSV nuevo con columnas:
% t_sec,
% pose_x, pose_y, pose_z,
% closest_x, closest_y, closest_z,
% err_x, err_y, err_z, err_norm,
% closest_seg_i, closest_seg_t

clear; close all; clc;

fname = "trajectory_errors.csv";

% Lee CSV saltando la cabecera
data = dlmread(fname, ",", 1, 0);

t        = data(:,1);  t = t - t(1);

pose_x   = data(:,2);  pose_y   = data(:,3);  pose_z   = data(:,4);
ref_x    = data(:,5);  ref_y    = data(:,6);  ref_z    = data(:,7);

err_x    = data(:,8);  err_y    = data(:,9);  err_z    = data(:,10);
err_norm = data(:,11);

seg_i    = data(:,12);
seg_t    = data(:,13);

% -------- Plot 1: norma del error vs tiempo
figure(1); clf;
plot(t, err_norm, "LineWidth", 1.5);
grid on;
xlabel("t [s]");
ylabel("||e|| [m]");
title("Trajectory tracking error norm (distance to reference polyline)");

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

% -------- Plot 3: trayectoria 3D (pose vs referencia más cercana)
figure(3); clf;
plot3(pose_x, pose_y, pose_z, "LineWidth", 1.5); hold on;
plot3(ref_x, ref_y, ref_z, "--", "LineWidth", 1.5);
grid on;
axis equal;
xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]");
legend("pose", "closest point on reference", "location", "best");
title("3D trajectory: pose vs reference (closest points)");

% (Opcional) Guardar a PNG
figure(1); print("-dpng", "err_norm_vs_time.png");
figure(2); print("-dpng", "err_xyz_vs_time.png");
figure(3); print("-dpng", "traj_3d_pose_vs_ref.png");

disp("Listo: generadas figuras y PNGs en la carpeta actual.");
