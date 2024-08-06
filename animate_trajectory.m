 
  % Load the simulation data
sim_data = dlmread('sim_data_static_envir_20240121.txt');
sim_info = load('sim_info_static_envir_20240121.txt');

dt = sim_info(1, 3);
num_obs = sim_info(1, 2);
num_drone = sim_info(1, 1);
dim_drone = sim_info(2, :);

a_drone = dim_drone(1);
b_drone = dim_drone(2);
c_drone = dim_drone(3);

init_drone = sim_info(3:2+num_drone, :);
goal_drone = sim_info(3+num_drone:2+2*num_drone, :);

pos_obs = sim_info(3+2*num_drone:2+2*num_drone+num_obs, :);
dim_obs = sim_info(3+2*num_drone+num_obs:end, :);

x_obs = pos_obs(:, 1);
y_obs = pos_obs(:, 2);
z_obs = pos_obs(:, 3);

a_obs = dim_obs(:, 1);
b_obs = dim_obs(:, 2);
c_obs = dim_obs(:, 3);



fig = figure(1);
set(fig,'Position',[100,100,1200,1200]);
set(gcf, 'Color', 'white');
ax = axes('Parent', fig, 'Projection', 'orthographic');
x_lim = [-2, 2];
y_lim = [-2, 2];
z_lim = [+0, 2.4];
title(ax, 'Trajectory');
xlabel(ax, 'x in m');
ylabel(ax, 'y in m');
zlabel(ax, 'z in m');
view(ax, [30, 45]);


phi_obs = linspace(0, 2*pi, 10)';
theta_obs = linspace(0, pi/2, 10);
% 
% phi_drone = linspace(0, 2*pi, 10)';
% theta_drone = linspace(0, pi, 10);
num_points = 10;
theta_drone = linspace(0, 2*pi, 10);
phi_drone = linspace(0, pi,10);
[theta_drone, phi_drone] = meshgrid(theta_drone, phi_drone);
colors = [
    0, 0, 1;    % 蓝色
    1, 0, 0;    % 红色
    0, 1, 0;    % 绿色
    1, 1, 0;    % 黄色
    0, 1, 1;    % 青色
    1, 0, 1;    % 洋红
    0.5, 0.5, 0.5;  % 灰色
    0, 0.5, 0;  % 橄榄绿
    0, 0, 0.5;  % 海军蓝
    0.5, 0, 0;  % 棕色
    0.5, 0, 0.5;  % 紫色
    0, 0.5, 0.5;  % 青绿
    0.5, 0.5, 0;  % 橙色
    0.15, 0.75, 0.75;  % 银色
    0.5, 0.5, 0.85;  % 灰色
    0.25, 0.25, 0.25;  % 暗灰色
    1, 0.84, 0;  % 金色
    1, 0.64, 0;  % 橙黄色
    0.5, 0.17, 0;  % 橄榄色
    1, 0.41, 0.71;  % 粉红
    0.7, 0.2, 0.2;  % 棕红
    0.6, 0.8, 1;  % 淡蓝色
    0, 0.39, 0;  % 深绿色
    0.22, 0, 0.22;  % 紫红
    1, 0.89, 0.77;  % 桃色
    0.75, 0.75, 0.75;  % 银色
    0.36, 0.25, 0.2;  % 栗色
    0.69, 0.77, 0.87;  % 淡青色
    0.6, 0.2, 0;  % 栗红
    0, 0.85, 0.15; % 翠绿
    0.74, 0.54, 0.27;  % 黄褐色
    0.48, 0.63, 0.86;  % 淡蓝灰
    0.96, 0.87, 0.7;  % 肉色
];


collision_count_agent = 0;
collision_count_obs = 0;
sim_steps = size(sim_data, 1) / (num_drone * 3);

for i = 1:sim_steps
    cla(ax);
    for k = 1:numel(x_obs)
        if c_obs(k) > z_lim(2)
            z = linspace(z_obs(k), 2.0, 20);
            theta = linspace(0, 2*pi, 20);
            [theta_obs, z_ell_obs] = meshgrid(theta, z);

            x_ell_obs = x_obs(k) + a_obs(k)* cos(theta_obs);
            y_ell_obs = y_obs(k) + b_obs(k)* sin(theta_obs);
        else
            x_ell_obs = x_obs(k) + a_obs(k) * sin(theta_obs) .* cos(phi_obs);
            y_ell_obs = y_obs(k) + b_obs(k) * sin(theta_obs) .* sin(phi_obs);
            z_ell_obs = z_obs(k) + c_obs(k) * cos(theta_obs);
        end
        %surf(ax, x_ell_obs, y_ell_obs, z_ell_obs, 'FaceColor', '#2980b9','EdgeColor', 'none', 'FaceAlpha', 0.4);
       new_x_obs = x_obs(k) ; % Update based on your movement logic
       new_y_obs = y_obs(k) ; % Update based on your movement logic
       new_z_obs = z_obs(k) ; % Update based on your movement logic
    if c_obs(k) > z_lim(2)
        z = linspace(new_z_obs, 2.0, 20);
        theta = linspace(0, 2*pi, 20);
        [theta_obs, z_ell_obs] = meshgrid(theta, z);

        x_ell_obs = new_x_obs + a_obs(k) * cos(theta_obs);
        y_ell_obs = new_y_obs + b_obs(k) * sin(theta_obs);
    else
        x_ell_obs = new_x_obs + a_obs(k) * sin(theta_obs) .* cos(phi_obs);
        y_ell_obs = new_y_obs + b_obs(k) * sin(theta_obs) .* sin(phi_obs);
        z_ell_obs = new_z_obs + c_obs(k) * cos(theta_obs);
    end
    x_obs(k) = new_x_obs;
    y_obs(k) = new_y_obs;
    z_obs(k) = new_z_obs;
    surf(ax, x_ell_obs, y_ell_obs, z_ell_obs, 'FaceColor', '#2980b9','EdgeColor', 'none', 'FaceAlpha', 0.4);
    end
    body = [];
    predictions = [];
    for k = 1:numel(x_obs)
        if c_obs(k) > z_lim(2)
            z = linspace(z_obs(k), 2.0, 20);
            theta = linspace(0, 2*pi, 20);
            [theta_obs, z_ell_obs] = meshgrid(theta, z);

            x_ell_obs = x_obs(k) + a_obs(k)* cos(theta_obs);
            y_ell_obs = y_obs(k) + b_obs(k)* sin(theta_obs);
        else
            x_ell_obs = x_obs(k) + a_obs(k) * sin(theta_obs) .* cos(phi_obs);
            y_ell_obs = y_obs(k) + b_obs(k) * sin(theta_obs) .* sin(phi_obs);
            z_ell_obs = z_obs(k) + c_obs(k) * cos(theta_obs);
        end
        surf(ax, x_ell_obs, y_ell_obs, z_ell_obs, 'FaceColor', '#2980b9','EdgeColor', 'none', 'FaceAlpha', 0.4);
        %%%%%%%%%%%%%%%%%%%%%
        radius_leaf = a_obs(k,1);  % 调整叶子的半径
        n_leaf = 100;      % 控制绘制的精细程度，可以根据需要调整

        % 创建球体的坐标
        theta_leaf = linspace(0, 2 * pi, n_leaf);
        phi_leaf = linspace(0, pi, n_leaf);
        [theta_leaf, phi_leaf] = meshgrid(theta_leaf, phi_leaf);

        x_leaf = radius_leaf * sin(phi_leaf) .* cos(theta_leaf);
        y_leaf = radius_leaf * sin(phi_leaf) .* sin(theta_leaf);
        z_leaf = radius_leaf * cos(phi_leaf);

        center_x_leaf = x_obs(k);
        center_y_leaf = y_obs(k);
        center_z_leaf = z_obs(k)+2.2;

        % 平移球体的坐标以改变圆心位置
        x_leaf = x_leaf + center_x_leaf;
        y_leaf = y_leaf + center_y_leaf;
        z_leaf = z_leaf + center_z_leaf;

        % 使用surf函数绘制绿色叶子
        
        surf(ax, x_leaf, y_leaf, z_leaf, 'FaceColor', '#0080a9', 'EdgeColor', 'none', 'FaceAlpha', 0.4);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        hold on;
    end
    x_data = sim_data((3*i-3)*num_drone + 1:(3*i-3)*num_drone + num_drone, :);
    y_data = sim_data((3*i-2)*num_drone + 1:(3*i-2)*num_drone + num_drone, :);
    z_data = sim_data((3*i-1)*num_drone + 1:(3*i-1)*num_drone + num_drone, :);

    for k = 1:num_drone
        
        x_ell_drone = x_data(k, 1) + a_drone * sin(theta_drone) .* cos(phi_drone);
        y_ell_drone = y_data(k, 1) + b_drone * sin(theta_drone) .* sin(phi_drone);
        z_ell_drone = z_data(k, 1) + c_drone * cos(theta_drone);
        
% 使用 surf 函数绘制三维曲面图
          
        body_temp = surf(ax, x_ell_drone, y_ell_drone, z_ell_drone,'FaceColor', colors(k, :), 'EdgeColor', 'none', 'FaceAlpha', 0.5);
        body = [body, body_temp];

        predictions_temp = plot3(ax, x_data(k, :), y_data(k, :), z_data(k, :), 'Color', colors(k, :), 'LineStyle', '-', 'LineWidth', 2);
        predictions = [predictions, predictions_temp];
    end
    xlim([-2.1, 2.1]);
ylim([-2.1, 2.1]);
zlim([-0.2, 2.4]);
% 框起XYZ轴和边
line(ax, [x_lim(1), x_lim(2)], [y_lim(1), y_lim(1)], [z_lim(1), z_lim(1)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 底部 X轴
line(ax, [x_lim(2), x_lim(2)], [y_lim(1), y_lim(2)], [z_lim(1), z_lim(1)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 底部 Y轴
line(ax, [x_lim(1), x_lim(1)], [y_lim(2), y_lim(2)], [z_lim(1), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 顶部 X轴
line(ax, [x_lim(1), x_lim(1)], [y_lim(2), y_lim(1)], [z_lim(1), z_lim(1)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 顶部 Y轴

line(ax, [x_lim(1), x_lim(1)], [y_lim(1), y_lim(1)], [z_lim(1), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 左侧 Z轴
line(ax, [x_lim(2), x_lim(2)], [y_lim(2), y_lim(2)], [z_lim(1), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 右侧 Z轴
line(ax, [x_lim(1), x_lim(2)], [y_lim(1), y_lim(1)], [z_lim(2), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 底部 Z轴
line(ax, [x_lim(1), x_lim(2)], [y_lim(2), y_lim(2)], [z_lim(2), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 顶部 Z轴

% 补充四条边
line(ax, [x_lim(1), x_lim(1)], [y_lim(1), y_lim(2)], [z_lim(2), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 左侧 Y轴
line(ax, [x_lim(2), x_lim(2)], [y_lim(1), y_lim(2)], [z_lim(2), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 右侧 Y轴
line(ax, [x_lim(2), x_lim(2)], [y_lim(1), y_lim(1)], [z_lim(1), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 底部 X轴
line(ax, [x_lim(1), x_lim(2)], [y_lim(2), y_lim(2)], [z_lim(1), z_lim(1)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 顶部 X轴
    %view(ax, [0.5, -0.5, 0.7]);
    view(ax, [0, 0, 1]);
    grid off;
    drawnow;
      pause(0.1);

    if collision_count_agent > 0 || collision_count_obs > 0
        break;
    end

    if i ~= sim_steps
        delete(body);
        delete(predictions);
%         delete(stats);
    end
end



% view(ax, [0, 0, 1]);
%grid on; % 添加网格线


% 框起XYZ轴和边

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Load the simulation data
 sim_data = dlmread('sim_data_36_dynamic_20240122.txt');
 sim_info = load('sim_info_36_dynamic_20240122.txt');
 sim_vel = dlmread('sim_data1_36_dynamic_20240122.txt');
 mpc = dlmread('sim_results_36_dynamic_20240122.txt');
 mpc_step = mpc(1,1);
dt = sim_info(1, 3);
num_obs = sim_info(1, 2);
num_drone = sim_info(1, 1);
dim_drone = sim_info(2, :);

a_drone = dim_drone(1);
b_drone = dim_drone(2);
c_drone = dim_drone(3);

init_drone = sim_info(3:2+num_drone, :);

num_dynamic_obs=4;

goal_drone = sim_info(3+num_drone:2+2*num_drone, :);

pos_obs = sim_info(3+2*num_drone:2+2*num_drone+num_obs, :);
dim_obs = sim_info(3+2*num_drone+num_obs:end, :);

pos_dynamic_obs =  sim_info(3+2*num_drone+2*num_obs:2+2*num_drone+2*num_obs+num_dynamic_obs,:);
dim_dynamic_obs = sim_info(3+2*num_drone+2*num_obs+num_dynamic_obs :2+2*num_drone+2*num_obs+2*num_dynamic_obs,:);
dynamic_obs_v = sim_info(3+2*num_drone+2*num_obs+2*num_dynamic_obs :2+2*num_drone+2*num_obs+3*num_dynamic_obs,:);

 x_dynamic_obs = pos_dynamic_obs(:,1);
 y_dynamic_obs = pos_dynamic_obs(:,2);
 z_dynamic_obs = pos_dynamic_obs(:,3);

 
a_dynamic_obs = dim_dynamic_obs(:,1);
b_dynamic_obs = dim_dynamic_obs(:,2);
c_dynamic_obs = dim_dynamic_obs(:,3);



x_obs = pos_obs(:, 1);
y_obs = pos_obs(:, 2);
z_obs = pos_obs(:, 3);

a_obs = dim_obs(:, 1);
b_obs = dim_obs(:, 2);
c_obs = dim_obs(:, 3);

x_lim = [-2, 2];
y_lim = [-2, 2];
z_lim = [+0, 2.4];

fig = figure(2);
set(fig,'Position',[100,100,1200,1200]);
set(gcf, 'Color', 'white');
ax = axes('Parent', fig, 'Projection', 'orthographic');

title(ax, 'Trajectory');
xlabel(ax, 'x in m');
ylabel(ax, 'y in m');
zlabel(ax, 'z in m');
view(ax, [30, 45]);


phi_obs = linspace(0, 2*pi, 10)';
theta_obs = linspace(0, pi/2, 10);
% 
% phi_drone = linspace(0, 2*pi, 10)';
% theta_drone = linspace(0, pi, 10);
num_points = 10;
theta_drone = linspace(0, 2*pi, 10);
phi_drone = linspace(0, pi,10);
[theta_drone, phi_drone] = meshgrid(theta_drone, phi_drone);
colors = [
    0, 0, 1;    % 蓝色
    1, 0, 0;    % 红色
    0, 1, 0;    % 绿色
    1, 1, 0;    % 黄色
    0, 1, 1;    % 青色
    1, 0, 1;    % 洋红
    0.5, 0.5, 0.5;  % 灰色
    0, 0.5, 0;  % 橄榄绿
    0, 0, 0.5;  % 海军蓝
    0.5, 0, 0;  % 棕色
    0.5, 0, 0.5;  % 紫色
    0, 0.5, 0.5;  % 青绿
    0.5, 0.5, 0;  % 橙色
    0.15, 0.75, 0.75;  % 银色
    0.5, 0.5, 0.85;  % 灰色
    0.25, 0.25, 0.25;  % 暗灰色
    1, 0.84, 0;  % 金色
    1, 0.64, 0;  % 橙黄色
    0.5, 0.17, 0;  % 橄榄色
    1, 0.41, 0.71;  % 粉红
    0.7, 0.2, 0.2;  % 棕红
    0.6, 0.8, 1;  % 淡蓝色
    0, 0.39, 0;  % 深绿色
    0.22, 0, 0.22;  % 紫红
    1, 0.89, 0.77;  % 桃色
    0.75, 0.75, 0.75;  % 银色
    0.36, 0.25, 0.2;  % 栗色
    0.69, 0.77, 0.87;  % 淡青色
    0.6, 0.2, 0;  % 栗红
    0, 0.85, 0.15; % 翠绿
    0.74, 0.54, 0.27;  % 黄褐色
    0.48, 0.63, 0.86;  % 淡蓝灰
    0.96, 0.87, 0.7;  % 肉色
];


collision_count_agent = 0;
collision_count_obs = 0;
sim_steps = size(sim_data, 1) / (num_drone * 3);

for i = 1:sim_steps
    cla(ax);
    for k = 1:numel(x_obs)
        if c_obs(k) > z_lim(2)
            z = linspace(z_obs(k), 2.0, 20);
            theta = linspace(0, 2*pi, 20);
            [theta_obs, z_ell_obs] = meshgrid(theta, z);

            x_ell_obs = x_obs(k) + a_obs(k)* cos(theta_obs);
            y_ell_obs = y_obs(k) + b_obs(k)* sin(theta_obs);
        else
            x_ell_obs = x_obs(k) + a_obs(k) * sin(theta_obs) .* cos(phi_obs);
            y_ell_obs = y_obs(k) + b_obs(k) * sin(theta_obs) .* sin(phi_obs);
            z_ell_obs = z_obs(k) + c_obs(k) * cos(theta_obs);
        end
        %surf(ax, x_ell_obs, y_ell_obs, z_ell_obs, 'FaceColor', '#2980b9','EdgeColor', 'none', 'FaceAlpha', 0.4);
       new_x_obs = x_obs(k) ; % Update based on your movement logic
       new_y_obs = y_obs(k) ; % Update based on your movement logic
       new_z_obs = z_obs(k) ; % Update based on your movement logic
    if c_obs(k) > z_lim(2)
        z = linspace(new_z_obs, 2.0, 20);
        theta = linspace(0, 2*pi, 20);
        [theta_obs, z_ell_obs] = meshgrid(theta, z);

        x_ell_obs = new_x_obs + a_obs(k) * cos(theta_obs);
        y_ell_obs = new_y_obs + b_obs(k) * sin(theta_obs);
    else
        x_ell_obs = new_x_obs + a_obs(k) * sin(theta_obs) .* cos(phi_obs);
        y_ell_obs = new_y_obs + b_obs(k) * sin(theta_obs) .* sin(phi_obs);
        z_ell_obs = new_z_obs + c_obs(k) * cos(theta_obs);
    end
    x_obs(k) = new_x_obs;
    y_obs(k) = new_y_obs;
    z_obs(k) = new_z_obs;
    surf(ax, x_ell_obs, y_ell_obs, z_ell_obs, 'FaceColor', '#2980b9','EdgeColor', 'none', 'FaceAlpha', 0.4);
    end
    body = [];
    obstacle = [];
    predictions = [];
    for k = 1:numel(x_obs)
        if c_obs(k) > z_lim(2)
            z = linspace(z_obs(k), 2.0, 20);
            theta = linspace(0, 2*pi, 20);
            [theta_obs, z_ell_obs] = meshgrid(theta, z);

            x_ell_obs = x_obs(k) + a_obs(k)* cos(theta_obs);
            y_ell_obs = y_obs(k) + b_obs(k)* sin(theta_obs);
        else
            x_ell_obs = x_obs(k) + a_obs(k) * sin(theta_obs) .* cos(phi_obs);
            y_ell_obs = y_obs(k) + b_obs(k) * sin(theta_obs) .* sin(phi_obs);
            z_ell_obs = z_obs(k) + c_obs(k) * cos(theta_obs);
        end
        surf(ax, x_ell_obs, y_ell_obs, z_ell_obs, 'FaceColor', '#2980b9','EdgeColor', 'none', 'FaceAlpha', 0.4);
         radius_leaf = a_obs(k,1);  % 调整叶子的半径
        n_leaf = 100;      % 控制绘制的精细程度，可以根据需要调整

        % 创建球体的坐标
        theta_leaf = linspace(0, 2 * pi, n_leaf);
        phi_leaf = linspace(0, pi, n_leaf);
        [theta_leaf, phi_leaf] = meshgrid(theta_leaf, phi_leaf);

        x_leaf = radius_leaf * sin(phi_leaf) .* cos(theta_leaf);
        y_leaf = radius_leaf * sin(phi_leaf) .* sin(theta_leaf);
        z_leaf = radius_leaf * cos(phi_leaf);

        center_x_leaf = x_obs(k);
        center_y_leaf = y_obs(k);
        center_z_leaf = z_obs(k)+2.2;

        % 平移球体的坐标以改变圆心位置
        x_leaf = x_leaf + center_x_leaf;
        y_leaf = y_leaf + center_y_leaf;
        z_leaf = z_leaf + center_z_leaf;

        % 使用surf函数绘制绿色叶子
        
        surf(ax, x_leaf, y_leaf, z_leaf, 'FaceColor', '#0080a9', 'EdgeColor', 'none', 'FaceAlpha', 0.4);
        hold on;
    end
    for k =1:num_dynamic_obs
                 z_top = 2.2;
                    z = linspace(z_dynamic_obs(k), z_top, 20);
                    theta = linspace(0, 2*pi, 20);
                    [theta_obs, z_ell_obs] = meshgrid(theta, z);

                    x_ell_obs = x_dynamic_obs(k) +dynamic_obs_v(k,1)*i+ a_dynamic_obs(k) * cos(theta_obs);
                    y_ell_obs = y_dynamic_obs(k) + dynamic_obs_v(k,2)*i+ b_dynamic_obs(k) * sin(theta_obs);
                  
                    surf(ax, x_ell_obs, y_ell_obs, z_ell_obs, 'FaceColor', [1,0,0], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
                            radius_leaf = a_dynamic_obs(k,1);  % 调整叶子的半径
        n_leaf = 100;      % 控制绘制的精细程度，可以根据需要调整

        % 创建球体的坐标
        theta_leaf = linspace(0, 2 * pi, n_leaf);
        phi_leaf = linspace(0, pi, n_leaf);
        [theta_leaf, phi_leaf] = meshgrid(theta_leaf, phi_leaf);

        x_leaf = radius_leaf * sin(phi_leaf) .* cos(theta_leaf);
        y_leaf = radius_leaf * sin(phi_leaf) .* sin(theta_leaf);
        z_leaf = radius_leaf * cos(phi_leaf);

        center_x_leaf = x_dynamic_obs(k) +dynamic_obs_v(k,1)*i;
        center_y_leaf = y_dynamic_obs(k) + dynamic_obs_v(k,2)*i;
        center_z_leaf = z_dynamic_obs(k)+2.2;
        
        % 平移球体的坐标以改变圆心位置
        x_leaf = x_leaf + center_x_leaf;
        y_leaf = y_leaf + center_y_leaf;
        z_leaf = z_leaf + center_z_leaf;
        obstacle_temp = surf(ax, x_leaf, y_leaf, z_leaf, 'FaceColor', [1,0,0], 'EdgeColor', 'none', 'FaceAlpha', 0.8);
        obstacle = [obstacle,obstacle_temp];
        % 使用surf函数绘制绿色叶子
               
                
    
            
                    
                
               
    end
    x_data = sim_data((3*i-3)*num_drone + 1:(3*i-3)*num_drone + num_drone, :);
    y_data = sim_data((3*i-2)*num_drone + 1:(3*i-2)*num_drone + num_drone, :);
    z_data = sim_data((3*i-1)*num_drone + 1:(3*i-1)*num_drone + num_drone, :);

    for k = 1:num_drone
        
        x_ell_drone = x_data(k, 1) + a_drone * sin(theta_drone) .* cos(phi_drone);
        y_ell_drone = y_data(k, 1) + b_drone * sin(theta_drone) .* sin(phi_drone);
        z_ell_drone = z_data(k, 1) + c_drone * cos(theta_drone);
        
% 使用 surf 函数绘制三维曲面图
       
        body_temp = surf(ax, x_ell_drone, y_ell_drone, z_ell_drone,'FaceColor', colors(k, :), 'EdgeColor', 'none', 'FaceAlpha', 0.5);
        body = [body, body_temp];

        predictions_temp = plot3(ax, x_data(k, :), y_data(k, :), z_data(k, :), 'Color', colors(k, :), 'LineStyle', '-', 'LineWidth', 2);
        predictions = [predictions, predictions_temp];
    end
    xlim([-2.1, 2.1]);
ylim([-2.1, 2.1]);
zlim([-0.2, 2.4]);
% 框起XYZ轴和边
line(ax, [x_lim(1), x_lim(2)], [y_lim(1), y_lim(1)], [z_lim(1), z_lim(1)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 底部 X轴
line(ax, [x_lim(2), x_lim(2)], [y_lim(1), y_lim(2)], [z_lim(1), z_lim(1)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 底部 Y轴
line(ax, [x_lim(1), x_lim(1)], [y_lim(2), y_lim(2)], [z_lim(1), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 顶部 X轴
line(ax, [x_lim(1), x_lim(1)], [y_lim(2), y_lim(1)], [z_lim(1), z_lim(1)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 顶部 Y轴

line(ax, [x_lim(1), x_lim(1)], [y_lim(1), y_lim(1)], [z_lim(1), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 左侧 Z轴
line(ax, [x_lim(2), x_lim(2)], [y_lim(2), y_lim(2)], [z_lim(1), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 右侧 Z轴
line(ax, [x_lim(1), x_lim(2)], [y_lim(1), y_lim(1)], [z_lim(2), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 底部 Z轴
line(ax, [x_lim(1), x_lim(2)], [y_lim(2), y_lim(2)], [z_lim(2), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 顶部 Z轴

% 补充四条边
line(ax, [x_lim(1), x_lim(1)], [y_lim(1), y_lim(2)], [z_lim(2), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 左侧 Y轴
line(ax, [x_lim(2), x_lim(2)], [y_lim(1), y_lim(2)], [z_lim(2), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 右侧 Y轴
line(ax, [x_lim(2), x_lim(2)], [y_lim(1), y_lim(1)], [z_lim(1), z_lim(2)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 底部 X轴
line(ax, [x_lim(1), x_lim(2)], [y_lim(2), y_lim(2)], [z_lim(1), z_lim(1)], 'Color', 'r', 'LineWidth', 1, 'LineStyle', '--'); % 顶部 X轴
    %view(ax, [0.5, -0.5, 0.7]);
    view(ax, [0,0,1]);
    grid off;
    drawnow;
      pause(0.1);

    if collision_count_agent > 0 || collision_count_obs > 0
        break;
    end

    if i ~= sim_steps
        delete(obstacle);
        delete(body);
        delete(predictions);
%         delete(stats);
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 sim_data = dlmread('sim_data_deadlock_20240122.txt');
sim_info = load('sim_info_deadlock_20240122.txt');
sim_vel = dlmread('sim_data1_deadlock_20240122.txt');
mpc = dlmread('sim_results_deadlock_20240122.txt');
mpc_step = mpc(1,1);
dt = sim_info(1, 3);
num_obs = sim_info(1, 2);
num_drone = sim_info(1, 1);
dim_drone = sim_info(2, :);

a_drone = dim_drone(1);
b_drone = dim_drone(2);
c_drone = dim_drone(3);

init_drone = sim_info(3:2+num_drone, :);


goal_drone = sim_info(3+num_drone:2+2*num_drone, :);

pos_obs = sim_info(3+2*num_drone:2+2*num_drone+num_obs, :);
dim_obs = sim_info(3+2*num_drone+num_obs:end, :);

x_obs = pos_obs(:, 1);
y_obs = pos_obs(:, 2);
z_obs = pos_obs(:, 3);

a_obs = dim_obs(:, 1);
b_obs = dim_obs(:, 2);
c_obs = dim_obs(:, 3);

x_lim = [-2, 2];
y_lim = [-1.5, 1.5];
z_lim = [+0, 2.4];
fig = figure(3);
set(fig,'Position',[100,100,1600,1200]);
set(gcf, 'Color', 'white');
ax = axes('Parent', fig, 'Projection', 'orthographic');

title(ax, 'Trajectory');
xlabel(ax, 'x in m');
ylabel(ax, 'y in m');
zlabel(ax, 'z in m');
view(ax, [30, 45]);


phi_obs = linspace(0, 2*pi, 10)';
theta_obs = linspace(0, pi/2, 10);
% 
% phi_drone = linspace(0, 2*pi, 10)';
% theta_drone = linspace(0, pi, 10);
num_points = 10;
theta_drone = linspace(0, 2*pi, 10);
phi_drone = linspace(0, pi,10);
[theta_drone, phi_drone] = meshgrid(theta_drone, phi_drone);
colors = [
    0.937, 0.505, 0.513;    % 蓝色
    0.984,0.706,0.365;    % 红色
   0.643,0.851,0.733;    % 绿色
   0.412,0.619,0.831;    % 黄色
    0.722,0.55,0.753;    % 青色
];


collision_count_agent = 0;
collision_count_obs = 0;
sim_steps = size(sim_data, 1) / (num_drone * 3);

for i = 1:sim_steps
    cla(ax);
    center = [0, 1.065, 1.2];
sideLength = 1.87;  % 正方体的边长
% 定义正方体的8个顶点坐标
vertices = [
    center(1) - sideLength/2, center(2) - sideLength/2, center(3) - sideLength/2;
    center(1) + sideLength/2, center(2) - sideLength/2, center(3) - sideLength/2;
    center(1) + sideLength/2, center(2) + sideLength/2, center(3) - sideLength/2;
    center(1) - sideLength/2, center(2) + sideLength/2, center(3) - sideLength/2;
    center(1) - sideLength/2, center(2) - sideLength/2, center(3) + sideLength/2;
    center(1) + sideLength/2, center(2) - sideLength/2, center(3) + sideLength/2;
    center(1) + sideLength/2, center(2) + sideLength/2, center(3) + sideLength/2;
    center(1) - sideLength/2, center(2) + sideLength/2, center(3) + sideLength/2
];

% 定义正方体的6个面
faces = [
    1, 2, 3, 4;
    5, 6, 7, 8;
    1, 2, 6, 5;
    2, 3, 7, 6;
    3, 4, 8, 7;
    4, 1, 5, 8
];

% 使用patch函数绘制正方体
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', '#d8dee3', 'EdgeColor', 'none');
center = [0, -1.065, 1.2];
sideLength = 1.87;  % 正方体的边长
% 定义正方体的8个顶点坐标
vertices = [
    center(1) - sideLength/2, center(2) - sideLength/2, center(3) - sideLength/2;
    center(1) + sideLength/2, center(2) - sideLength/2, center(3) - sideLength/2;
    center(1) + sideLength/2, center(2) + sideLength/2, center(3) - sideLength/2;
    center(1) - sideLength/2, center(2) + sideLength/2, center(3) - sideLength/2;
    center(1) - sideLength/2, center(2) - sideLength/2, center(3) + sideLength/2;
    center(1) + sideLength/2, center(2) - sideLength/2, center(3) + sideLength/2;
    center(1) + sideLength/2, center(2) + sideLength/2, center(3) + sideLength/2;
    center(1) - sideLength/2, center(2) + sideLength/2, center(3) + sideLength/2
];

% 定义正方体的6个面
faces = [
    1, 2, 3, 4;
    5, 6, 7, 8;
    1, 2, 6, 5;
    2, 3, 7, 6;
    3, 4, 8, 7;
    4, 1, 5, 8
];

% 使用patch函数绘制正方体
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', '#d8dee3', 'EdgeColor', 'none');

center = [0, 1.2, 1.2];
sideLength = 2.34;  % 正方体的边长
sidewidth = 1.6;  % 正方体的边长
% 定义正方体的8个顶点坐标
vertices = [
    center(1) - sideLength/2, center(2) - sidewidth/2, center(3) - 1.88/2;
    center(1) + sideLength/2, center(2) - sidewidth/2, center(3) - 1.88/2;
    center(1) + sideLength/2, center(2) + sidewidth/2, center(3) - 1.88/2;
    center(1) - sideLength/2, center(2) + sidewidth/2, center(3) - 1.88/2;
    center(1) - sideLength/2, center(2) - sidewidth/2, center(3) + 1.88/2;
    center(1) + sideLength/2, center(2) - sidewidth/2, center(3) + 1.88/2;
    center(1) + sideLength/2, center(2) + sidewidth/2, center(3) + 1.88/2;
    center(1) - sideLength/2, center(2) + sidewidth/2, center(3) + 1.88/2
];

% 定义正方体的6个面
faces = [
    1, 2, 3, 4;
    5, 6, 7, 8;
    1, 2, 6, 5;
    2, 3, 7, 6;
    3, 4, 8, 7;
    4, 1, 5, 8
];

% 使用patch函数绘制正方体
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', '#d8dee3', 'EdgeColor', 'none');

center = [0, -1.2, 1.2];
sideLength = 2.34;  % 正方体的边长
sidewidth = 1.6;  % 正方体的边长
% 定义正方体的8个顶点坐标
vertices = [
    center(1) - sideLength/2, center(2) - sidewidth/2, center(3) - 1.88/2;
    center(1) + sideLength/2, center(2) - sidewidth/2, center(3) - 1.88/2;
    center(1) + sideLength/2, center(2) + sidewidth/2, center(3) - 1.88/2;
    center(1) - sideLength/2, center(2) + sidewidth/2, center(3) - 1.88/2;
    center(1) - sideLength/2, center(2) - sidewidth/2, center(3) + 1.88/2;
    center(1) + sideLength/2, center(2) - sidewidth/2, center(3) + 1.88/2;
    center(1) + sideLength/2, center(2) + sidewidth/2, center(3) + 1.88/2;
    center(1) - sideLength/2, center(2) + sidewidth/2, center(3) + 1.88/2
];

% 定义正方体的6个面
faces = [
    1, 2, 3, 4;
    5, 6, 7, 8;
    1, 2, 6, 5;
    2, 3, 7, 6;
    3, 4, 8, 7;
    4, 1, 5, 8
];

% 使用patch函数绘制正方体
patch('Vertices', vertices, 'Faces', faces, 'FaceColor', '#d8dee3', 'EdgeColor', 'none');
    for k = 1:numel(x_obs)
        if c_obs(k) > z_lim(2)
            z = linspace(z_obs(k), 2.0, 20);
            theta = linspace(0, 2*pi, 20);
            [theta_obs, z_ell_obs] = meshgrid(theta, z);

            x_ell_obs = x_obs(k) + a_obs(k)* cos(theta_obs);
            y_ell_obs = y_obs(k) + b_obs(k)* sin(theta_obs);
        else
            x_ell_obs = x_obs(k) + a_obs(k) * sin(theta_obs) .* cos(phi_obs);
            y_ell_obs = y_obs(k) + b_obs(k) * sin(theta_obs) .* sin(phi_obs);
            z_ell_obs = z_obs(k) + c_obs(k) * cos(theta_obs);
        end
        %surf(ax, x_ell_obs, y_ell_obs, z_ell_obs, 'FaceColor', '#2980b9','EdgeColor', 'none', 'FaceAlpha', 0.4);
       new_x_obs = x_obs(k) ; % Update based on your movement logic
       new_y_obs = y_obs(k) ; % Update based on your movement logic
       new_z_obs = z_obs(k) ; % Update based on your movement logic
    if c_obs(k) > z_lim(2)
        z = linspace(new_z_obs, 2.0, 20);
        theta = linspace(0, 2*pi, 20);
        [theta_obs, z_ell_obs] = meshgrid(theta, z);

        x_ell_obs = new_x_obs + a_obs(k) * cos(theta_obs);
        y_ell_obs = new_y_obs + b_obs(k) * sin(theta_obs);
    else
        x_ell_obs = new_x_obs + a_obs(k) * sin(theta_obs) .* cos(phi_obs);
        y_ell_obs = new_y_obs + b_obs(k) * sin(theta_obs) .* sin(phi_obs);
        z_ell_obs = new_z_obs + c_obs(k) * cos(theta_obs);
    end
    x_obs(k) = new_x_obs;
    y_obs(k) = new_y_obs;
    z_obs(k) = new_z_obs;
    surf(ax, x_ell_obs, y_ell_obs, z_ell_obs, 'FaceColor', '#2980b9','EdgeColor', 'none', 'FaceAlpha', 0.4);
    end
    body = [];
    predictions = [];
    for k = 1:numel(x_obs)
        if c_obs(k) > z_lim(2)
            z = linspace(z_obs(k), 2.0, 20);
            theta = linspace(0, 2*pi, 20);
            [theta_obs, z_ell_obs] = meshgrid(theta, z);

            x_ell_obs = x_obs(k) + a_obs(k)* cos(theta_obs);
            y_ell_obs = y_obs(k) + b_obs(k)* sin(theta_obs);
        else
            x_ell_obs = x_obs(k) + a_obs(k) * sin(theta_obs) .* cos(phi_obs);
            y_ell_obs = y_obs(k) + b_obs(k) * sin(theta_obs) .* sin(phi_obs);
            z_ell_obs = z_obs(k) + c_obs(k) * cos(theta_obs);
        end
        surf(ax, x_ell_obs, y_ell_obs, z_ell_obs, 'FaceColor', '#2980b9','EdgeColor', 'none', 'FaceAlpha', 0.4);
                radius_leaf = a_obs(k,1);  % 调整叶子的半径
        n_leaf = 100;      % 控制绘制的精细程度，可以根据需要调整

        % 创建球体的坐标
        theta_leaf = linspace(0, 2 * pi, n_leaf);
        phi_leaf = linspace(0, pi, n_leaf);
        [theta_leaf, phi_leaf] = meshgrid(theta_leaf, phi_leaf);

        x_leaf = radius_leaf * sin(phi_leaf) .* cos(theta_leaf);
        y_leaf = radius_leaf * sin(phi_leaf) .* sin(theta_leaf);
        z_leaf = radius_leaf * cos(phi_leaf);

        center_x_leaf = x_obs(k);
        center_y_leaf = y_obs(k);
        center_z_leaf = z_obs(k)+2.2;

        % 平移球体的坐标以改变圆心位置
        x_leaf = x_leaf + center_x_leaf;
        y_leaf = y_leaf + center_y_leaf;
        z_leaf = z_leaf + center_z_leaf;

        % 使用surf函数绘制绿色叶子
        
        surf(ax, x_leaf, y_leaf, z_leaf, 'FaceColor', '#d8dee3', 'EdgeColor', 'none', 'FaceAlpha', 1);
       
  
%     end

        hold on;
    end
    
    x_data = sim_data((3*i-3)*num_drone + 1:(3*i-3)*num_drone + num_drone, :);
    y_data = sim_data((3*i-2)*num_drone + 1:(3*i-2)*num_drone + num_drone, :);
    z_data = sim_data((3*i-1)*num_drone + 1:(3*i-1)*num_drone + num_drone, :);

    for k = 1:num_drone
        
        x_ell_drone = x_data(k, 1) + a_drone * sin(theta_drone) .* cos(phi_drone);
        y_ell_drone = y_data(k, 1) + b_drone * sin(theta_drone) .* sin(phi_drone);
        z_ell_drone = z_data(k, 1) + c_drone * cos(theta_drone);
        
% 使用 surf 函数绘制三维曲面图
          
        body_temp = surf(ax, x_ell_drone, y_ell_drone, z_ell_drone,'FaceColor', colors(k, :), 'EdgeColor', 'none', 'FaceAlpha', 0.5);
        body = [body, body_temp];

        predictions_temp = plot3(ax, x_data(k, :), y_data(k, :), z_data(k, :), 'Color', colors(k, :), 'LineStyle', '-', 'LineWidth', 2);
        predictions = [predictions, predictions_temp];
    end
    xlim([-2.1, 2.1]);
ylim([-1.5, 1.5]);
zlim([-0.2, 2.4]);
% 框起XYZ轴和边

    view(ax, [0, 0, 1]);
    grid off;
    drawnow;
      pause(0.1);

    if collision_count_agent > 0 || collision_count_obs > 0
        break;
    end

    if i ~= sim_steps
        delete(body);
        delete(predictions);
%         delete(stats);
    end
end
