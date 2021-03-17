%%
leaves_pos = [];
save('test.mat', 'leaves_pos')
fig=figure;
hold on
axis tight

% tree = pcread('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/mega_cloud16.pcd');
% tfmat = eye(4);
% tfmat(1:3,1:3) = roty(-90);
% tform = affine3d(tfmat);


% tree = pcread('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/skeletonization_data/SkeletonEvaluation/generated/ply/tree1_28189.ply');
tree = pcread('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/skeletonization_data/SkeletonEvaluation/realdata/ply/tree1_6678.ply');
tfmat = eye(4);
tfmat(1:3,1:3) = rotx(-90);
tform = affine3d(tfmat);
ptcloud1 = pctransform(tree, tform);
pcshow(ptcloud1)

% dcm_obj = datacursormode(fig);
% set(dcm_obj,'UpdateFcn',@myupdatefcn1)