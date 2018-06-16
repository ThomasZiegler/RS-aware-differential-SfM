function [R, T] = calculate_camera_trajectory(height, R_0, T_0, gamma, w_0, v_0, k)
    %% Set fixed parameters
    T_a = gamma;
    T_b = 1-gamma;
    
    w_rad = w_0 * pi / 180;  % Convert from degrees to radians
        
    R = R_0;
    T = T_0;
    
    w_skew = [0 -w_rad(3) w_rad(2);
         w_rad(3) 0 -w_rad(1);
        -w_rad(2) w_rad(1) 0];

    beta = ones(height, 2);
    for i=1:length(beta)
        beta_1 = 2.0/(2.0+k)* (gamma*i/height + 0.5*k*(gamma*i/height)^2);
        beta_2 = 2.0/(2.0+k)* (1.0 + gamma*i/height + 0.5*k*(1.0 + gamma*i/height)^2);
        beta(i,:) = [beta_1, beta_2];
    end
    beta = reshape(beta, [2*height 1]);

    for i=1:length(beta)-1
        R = [R; [R_0 * (eye(3,3) + beta(i+1)*w_skew)]];
        T = [T; [T_0 + beta(i+1)*v_0']];
    end
end
