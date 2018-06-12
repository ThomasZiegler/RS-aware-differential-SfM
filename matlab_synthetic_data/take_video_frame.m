function take_video_frame(dir, model, position, width, height, A, R, T)
    % Rendering
    [init_depth, init_gs, unproject, A, R, T] = renderer(width, height, model, 0, 0, A, R, T);

    % Normalize depth map
    norm_depth = init_depth - min(init_depth(:));
    norm_depth = norm_depth ./ max(norm_depth(:));

    % Save initial rendering
    imwrite(init_gs, strcat(dir, position, '_gs.png'));
    imwrite(norm_depth, strcat(dir, position, '_depth.png'));
end