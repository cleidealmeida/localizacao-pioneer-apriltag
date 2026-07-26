function I_out = mDrawAxes(tag, I)
    if isempty(tag.pCamParams), error('Calibration required'); end
    
    intrinsics = tag.pCamParams.Intrinsics;
    H = tag.pPar.H;
    len = tag.pPar.size / 2;
    
    pts3d_tag = [0,0,0; len,0,0; 0,len,0; 0,0,len];
    R = H(1:3, 1:3);
    t = H(1:3, 4);
    pts3d_cam = (R * pts3d_tag' + t)'; 
    
    pts2d = worldToImage(intrinsics, eye(3), [0 0 0], pts3d_cam);
    
    I_out = insertShape(I, 'Line', [pts2d(1,:), pts2d(2,:)], 'Color', 'red', 'LineWidth', 5);
    I_out = insertShape(I_out, 'Line', [pts2d(1,:), pts2d(3,:)], 'Color', 'green', 'LineWidth', 5);
    I_out = insertShape(I_out, 'Line', [pts2d(1,:), pts2d(4,:)], 'Color', 'blue', 'LineWidth', 5);
    imshow(I_out); title(['ID: ' num2str(tag.pPar.id)]);
end