function [A, R, T, mean_depth] = setup_renderer(dir, model, width, height, angles)
    % First rendering to get A, R, T
    [init_depth, init_gs, ~, A, R, T] = renderer(width, height, model, 0, 0, 1, angles(1), angles(2), angles(3), 'zxy');
    imwrite(init_gs, strcat(dir, 'setup_gs.png'));
    
    % Move camera and re-render
    % + left / - right
    % + up / - down
    % + backwards / - forwards
    init_delta_T = [2, -9, -114.8];
    
    % Use init_delta_T = [3, -10, -114.8] for small gamma values (0-0.3) with different start position
    % Use init_delta_T = [-6, -11, -112] for "Vertical Lines"
    
    T = T + init_delta_T;
    [init_depth, init_gs, unproject, A, R, T] = renderer(width, height, model, 0, 0, A, R, T);
    
    % Save initial rendering
    imwrite(init_gs, strcat(dir, 'T_setup_gs.png'));
    
    % Transform to camera coordinates to rotate camera
    r_x = 0.0; % + up / - down
    r_y = 0.0; % + right / - left
    r_z = -0.2; % + Counter-clockwise / - clockwise

%    % Use these values for "Vertical Lines":
%     r_x = 0.3; % + up / - down
%     r_y = -0.6; % + right / - left
%     r_z = 0.63; % + Counter-clockwise / - clockwise

    R_x = [1 0 0; 0 cos(r_x) -sin(r_x); 0 sin(r_x) cos(r_x)];
    R_y = [cos(r_y) 0 sin(r_y); 0 1 0; -sin(r_y) 0 cos(r_y)];
    R_z = [cos(r_z) -sin(r_z) 0; sin(r_z) cos(r_z) 0; 0 0 1];
    
    R_c = R';
    C = - inv(R) * T';
    
    init_delta_R = R_x * R_y * R_z;
    R_c = R_c * init_delta_R;
    
    % Transform back to world coordinates
    R = R_c';
    T = (-R*C)';
    
    % Re-render
    [init_depth, init_gs, unproject, A, R, T] = renderer(width, height, model, 0, 0, A, R, T);
    
    % Save initial rendering
    imwrite(init_gs, strcat(dir, 'R_setup_gs.png'));
    
    mean_depth = abs(mean(mean(unproject(:,:,3))));
end