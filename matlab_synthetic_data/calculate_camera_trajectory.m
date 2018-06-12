function [R, T] = calculate_camera_trajectory(height, R_0, T_0, gamma, w_0, v_0, k, method)
    %% Set fixed parameters
    T_a = gamma;
    T_b = 1-gamma;
    
    w_rad = w_0 * pi / 180;  % Convert from degrees to radians
        
    R = R_0;
    T = T_0;
    
    w_skew = [0 -w_rad(3) w_rad(2);
         w_rad(3) 0 -w_rad(1);
        -w_rad(2) w_rad(1) 0];
    
    %% Calculate trajectory based on chosen method
    if method == 'beta'
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
    
    if method == 'time'
        time_rs1 = linspace(0,T_a,height);
        time_rs2 = linspace(T_a+T_b,2*T_a+T_b,height);
        time = [time_rs1, time_rs2];
    
        %% Calculate rotation in world coordinates
        w = repmat(w_rad, 1, length(time)) + k * w_rad * time;

        for i=1:length(time)-1
            w_skew = [0 -w(3,i) w(2,i);
                      w(3,i) 0 -w(1,i);
                     -w(2,i) w(1,i) 0];
            R_old = R(3*i-2:3*i,:);
            T_old = T(i,:);
            R = [R; [(eye(3,3) + w_skew)*R_old]]; %This approximates exp(w_skew)*R_old
            C_old = -inv(R_old) * T_old';
            R_new = R(3*i+1:3*i+3,:);
            T = [T; [-R_new*C_old]'];
        end
    
        %% Calculate translation in world coordinates
        T = T' + v_0 * time + 1/2 * k * v_0 * time.^2;
        T = T';
    end
end
