function take_single_frame(dir, model, position, width, height, A, R, T)
    % Rendering
    [init_depth, init_gs, unproject, A, R, T] = renderer(width, height, model, 0, 0, A, R, T);

    % Normalize depth map
    norm_depth = init_depth - min(init_depth(:));
    norm_depth = norm_depth ./ max(norm_depth(:));

    % Save initial rendering
    imwrite(init_gs, strcat(dir, position, '_gs.png'));
    imwrite(norm_depth, strcat(dir, position, '_depth.png'));

    % Save initial matrices as .csv
    csvwrite(strcat(dir, position, '_gs_t.csv'), T);
    csvwrite(strcat(dir, position, '_gs_r.csv'), reshape(R', [1 9]));
    csvwrite(strcat(dir, 'A.csv'), A);
    csvwrite(strcat(dir, position, '_gs_unproject_x.csv'), unproject(:,:,1));
    csvwrite(strcat(dir, position, '_gs_unproject_y.csv'), unproject(:,:,2));
    csvwrite(strcat(dir, position, '_gs_unproject_z.csv'), unproject(:,:,3));

    % Save initial matrices as .mat
    save(strcat(dir, position, '_non_normalized_depth.mat'), 'init_depth');
    save(strcat(dir, position, '.mat'), 'norm_depth', 'init_gs', 'unproject', 'A', 'R', 'T');
end