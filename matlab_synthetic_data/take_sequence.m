function take_sequence(dir, model, width, height, A, R, T, take_video)
    %% Take first GS image
    take_single_frame(dir, model, '1_initial', width, height, A, R(1:3,:), T(1,:));
    
    %% Take first RS image
    final_rs_1 = zeros(width, height, 3, 'uint8');
    final_depth_1 = zeros(width, height, 1, 'double');
    final_unproject_1 = zeros(width, height, 3, 'double');
    
    for i=1:height
        i
        R_i = R(3*i-2:3*i, :);
        T_i = T(i, :);
        [partial_depth_1, partial_rs_1, partial_unproject_1, ~, ~, ~] = renderer(width, height, model, 0, 0, A, R_i, T_i);
        final_rs_1(i,:,:) = partial_rs_1(i,:,:);
        final_depth_1(i,:,:) = partial_depth_1(i,:,:);
        final_unproject_1(i,:,:) = partial_unproject_1(i,:,:);
        
        if take_video == 1
            % Take video (only use every 5th frame)
            if mod(i,5) == 0
                take_video_frame(dir, model, num2str(i), width, height, A, R_i, T_i);
            end
        end
    end
    
    %% Store RS image and other data
    R_write = reshape(R(1:3, :)', [1 9]);
    for i=2:height
        R_write = [R_write; reshape(R(3*i-2:3*i, :)', [1 9])];
    end
    csvwrite(strcat(dir, '1', '_rs_r.csv'), R_write);
    csvwrite(strcat(dir, '1', '_rs_t.csv'), T(1:height, :));

    csvwrite(strcat(dir, '1', '_rs_unproject_x.csv'), final_unproject_1(:,:,1));
    csvwrite(strcat(dir, '1', '_rs_unproject_y.csv'), final_unproject_1(:,:,2));
    csvwrite(strcat(dir, '1', '_rs_unproject_z.csv'), final_unproject_1(:,:,3));

    imwrite(final_rs_1, strcat(dir, '1', '_rs.png'));
    
    % Normalize depth map
    norm_final_depth_1 = final_depth_1 - min(final_depth_1(:));
    norm_final_depth_1 = norm_final_depth_1 ./ max(norm_final_depth_1(:));
    
    imwrite(norm_final_depth_1, strcat(dir, '1', '_rs_depth.png'));
    
    %% Take second GS image
    take_single_frame(dir, model, '1_final', width, height, A, R(3*height-2:3*height,:), T(height,:));
    
    %% Take first GS image
    take_single_frame(dir, model, '2_initial', width, height, A, R(3*height+1:3*height+3,:), T(height+1,:));
    
    %% Take second RS image
    final_rs_2 = zeros(width, height, 3, 'uint8');
    final_depth_2 = zeros(width, height, 1, 'double');
    final_unproject_2 = zeros(width, height, 3, 'double');
    
    for i=1:height
        j = height + i
        R_i = R(3*j-2:3*j, :);
        T_i = T(j, :);
        [partial_depth_2, partial_rs_2, partial_unproject_2, ~, ~, ~] = renderer(width, height, model, 0, 0, A, R_i, T_i);
        final_rs_2(i,:,:) = partial_rs_2(i,:,:);
        final_depth_2(i,:,:) = partial_depth_2(i,:,:);
        final_unproject_2(i,:,:) = partial_unproject_2(i,:,:);
        
        if take_video == 1
            % Take video (only use every 5th frame)
            if mod(i,5) == 0
                take_video_frame(dir, model, num2str(j), width, height, A, R_i, T_i);
            end
        end
    end
    
    %% Store RS image and other data
    R_write = reshape(R(height+1:height+3, :)', [1 9]);
    for i=2:height
        j = height + i;
        R_write = [R_write; reshape(R(3*j-2:3*j, :)', [1 9])];
    end
    csvwrite(strcat(dir, '2', '_rs_r.csv'), R_write);
    csvwrite(strcat(dir, '2', '_rs_t.csv'), T(height+1:2*height, :));

    csvwrite(strcat(dir, '2', '_rs_unproject_x.csv'), final_unproject_2(:,:,1));
    csvwrite(strcat(dir, '2', '_rs_unproject_y.csv'), final_unproject_2(:,:,2));
    csvwrite(strcat(dir, '2', '_rs_unproject_z.csv'), final_unproject_2(:,:,3));

    imwrite(final_rs_2, strcat(dir, '2', '_rs.png'));
    
    % Normalize depth map
    norm_final_depth_2 = final_depth_2 - min(final_depth_2(:));
    norm_final_depth_2 = norm_final_depth_2 ./ max(norm_final_depth_2(:));
    
    imwrite(norm_final_depth_2, strcat(dir, '2', '_rs_depth.png'));
    
    %% Take second GS image
    take_single_frame(dir, model, '2_final', width, height, A, R(3*2*height-2:3*2*height,:), T(2*height,:));
end