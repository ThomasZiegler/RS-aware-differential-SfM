function start_generating(width, gamma_values, k_values, w_values, v_values)
    % Take video (store renderings from all camera positions)
    take_video = 0;
    path_to_store = '~/generated_data/';
    
    height = width;  % assuming quadratic images
    
    for k_index = 1:length(k_values)
        k = k_values(k_index);

        for gamma_index = 1:length(gamma_values)
            gamma = gamma_values(gamma_index);

            for w_index = 1:length(w_values)
                w_0_rs = w_values(w_index);

                for v_index = 1:length(v_values)
                    v_0_magn = v_values(v_index); % normalized translation

                    %% Set rotation (only around z axis)
                    w_0 = [0; 0; w_0_rs];

                    %% Set translation 
                    v_0 = [v_0_magn; v_0_magn; 0]; % Use v_0 = [v_0_magn; 0; 0] for "Vertical Lines" 
                    
                    
                    %% Set output folder and model
                    dir = strcat('_v=', mat2str(v_0), '_w=', mat2str(w_0), '_k=', num2str(k), '_gamma=', num2str(gamma), '/');
                    dir = strcat(path_to_store, 'test_', dir);
                    model = 'landesmuseum.osg';
                    
                    [status, msg, msgID] = mkdir(sprintf('%s', dir));
                    if strcmp(msg, 'Directory already exists.') ==  1
                        disp('Skipping configuration because it was already rendered')
                        continue;
                    end
                    
                    %% Write parameters to csv files
                    csvwrite(strcat(dir, 'gamma.csv'), gamma);
                    csvwrite(strcat(dir, 'k.csv'), k);
                    csvwrite(strcat(dir, 'v.csv'), v_0');
                    csvwrite(strcat(dir, 'w.csv'), w_0');
                    
                    %% Store images one folder below
                    dir = strcat(dir, 'images/');
                    mkdir(sprintf('%s', dir));                    
                  
                    %% Setup renderer
                    % + up / - down
                    % + right / - left
                    % + clockwise / - counter-clockwise
                    angles = [166, 0.5, 35]; 

                    [A, R_0, T_0, mean_depth] = setup_renderer(dir, model, width, height, angles);

                    % Scale normalized translation according to average depth in the scene
                    v_0 = mean_depth * v_0;
                    csvwrite(strcat(dir, 'true_v.csv'), v_0');
                    
                    %% Calculate trajectory of camera
                    [R, T] = calculate_camera_trajectory(height, R_0, T_0, gamma, w_0, v_0, k);

                    %% Take RS images
                    take_sequence(dir, model, width, height, A, R, T, take_video);
                end
            end
        end
    end
end