
exportSimpleFisheye(fisheyeIntrinsics,'fish_eye_calibration.yml')


function exportSimpleFisheye(fisheyeIntrinsics, output_file)
    % 简单导出，焦距从StretchMatrix获取
    fx = fisheyeIntrinsics.StretchMatrix(1,1);
    fy = fisheyeIntrinsics.StretchMatrix(2,2);
    cx = fisheyeIntrinsics.DistortionCenter(1);
    cy = fisheyeIntrinsics.DistortionCenter(2);

    K = [fx, 0, cx; 0, fy, cy; 0, 0, 1];
    K

    % 畸变参数 - 这里可能需要调整
    % 尝试使用MappingCoefficients的前4个
    D = fisheyeIntrinsics.MappingCoefficients(1:4);

    % 归一化处理
    D = D * 1e-3;  % 缩放因子，需要根据实际情况调整
    D

    % 导出
    fid = fopen(output_file, 'w');
    fprintf(fid, '%%YAML:1.0\n\n');

    fprintf(fid, 'K: !!opencv-matrix\n');
    fprintf(fid, '  rows: 3\n  cols: 3\n  dt: d\n');
    fprintf(fid, '  data: [%.10f, 0, %.10f, 0, %.10f, %.10f, 0, 0, 1]\n\n', ...
        fx, cx, fy, cy);

    fprintf(fid, 'D: !!opencv-matrix\n');
    fprintf(fid, '  rows: 1\n  cols: 4\n  dt: d\n');
    fprintf(fid, '  data: [%.10f, %.10f, %.10f, %.10f]\n\n', ...
        D(1), D(2), D(3), D(4));

    fprintf(fid, 'image_width: %d\n', fisheyeIntrinsics.ImageSize(2));
    fprintf(fid, 'image_height: %d\n', fisheyeIntrinsics.ImageSize(1));

    fclose(fid);
end