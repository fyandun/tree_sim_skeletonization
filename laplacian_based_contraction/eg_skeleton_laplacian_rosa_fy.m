% function [] = eg_point_cloud_curve_skeleton(filename)
% extract curve skeleton from a point cloud or triangular mesh
% update: 2010-8-19
% update: 2010-7-12
% create: 2009-4-26
% by: JJCAO, deepfish @ DUT
%
%% setting
clear
clc
close all
path('toolbox',path);
options.USING_POINT_RING = GS.USING_POINT_RING;


%% Step 0: read file (point cloud & local feature size if possible), and
tic
% normalize the modle.
vines_synthetic = 1; %1 vines, 0 synthetic

if vines_synthetic
    %%SYNTHETIC DATA
%     test_file = 'mega_cloud1_lab_2';
    %test_file = 'mega_cloud_lab_new_2';
    test_file = 'mega_cloud_lab_complex';
    filename = strcat('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/', test_file);% pcd
%     test_file = 'mega_cloud_4';
%     filename = strcat('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/new_experiments/', test_file);% pcd
    k = strfind(test_file, 'lab');
    if isempty(k)
        rotm = roty(90);
        top_limit = 0.1;
        subsample_rate = 0.01; %0.02 for the vines before RL
    else
        rotm = rotx(-90);
        top_limit = 0.05;
        subsample_rate = 0.005; %0.01
    end
    extension='.pcd';
    
else
    %%SYNTHETIC DATA
    test_path = '/home/fyandun/Documentos/Manipulator/Data/various_dataSets/skeletonization_data/SkeletonEvaluation/generated/ply/';
    test_case_id = 6;
    filenames = {'tree1_noise_3', 'tree2_noise_3', 'tree3_noise_3', 'tree4_noise_3', 'tree5_noise_3', 'tree6_noise_3'};
    test_file = filenames{test_case_id}; 
    extension = '.ply';
    filename = strcat(test_path, test_file);    
    rotm = rotx(90);
    top_limit = 0.5; %1
    subsample_rate = 0.05;
end

% filename = '/home/fyandun/Documentos/Manipulator/Data/various_dataSets/skeletonization_data/SkeletonEvaluation/realdata/ply/tree1_6678';% ply
% rotm = rotx(90);
% top_limit = 0.5;

ptCloud_ = pcread(strcat(filename, extension));
if ptCloud_.Count >1e4
    ptCloudOut = pcdownsample(ptCloud_,'gridAverage',subsample_rate);
else
    ptCloudOut = ptCloud_;
end

% pcshow(ptCloudOut)
P.pts = double(ptCloudOut.Location);
P.faces = [];
P.npts = ptCloudOut.Count;
P.radis = ones(P.npts,1);

P.pts = transpose(rotm*P.pts'); %esto depende de si la nube de puntos está rotada
figure
hold on
scatter3(P.pts(:,1),P.pts(:,2),P.pts(:,3),20,'.');

stem_base = min(P.pts(:,3));
aux = P.pts(:,3)>stem_base & P.pts(:,3)<stem_base+top_limit;
xyz_base = mean(P.pts(aux,:));

plot3(xyz_base(1), xyz_base(2), xyz_base(3), 'xr')

[P.pts, scale] = GS.normalize(P.pts);

[P.bbox, P.diameter] = GS.compute_bbox(P.pts);
disp(sprintf('read point set:'));
toc

%% Step 1: build local 1-ring
% build neighborhood, knn?
tic
P.k_knn = GS.compute_k_knn(P.npts);
if options.USING_POINT_RING
    P.rings = compute_point_point_ring(P.pts, P.k_knn, []);
else    
    P.frings = compute_vertex_face_ring(P.faces);
    P.rings = compute_vertex_ring(P.faces, P.frings);
end
disp(sprintf('compute local 1-ring:'));
toc

%% Step 1: Contract point cloud by Laplacian
tic
[P.cpts, t, initWL, WC, sl] = contraction_by_mesh_laplacian(P, options);
fprintf('Contraction:\n');
toc
figure()
scatter3(P.cpts(:,1),P.cpts(:,2),P.cpts(:,3),30,'.r');

%%
P_orig = P;
if vines_synthetic
    out_path = '/home/fyandun/Documentos/Manipulator/Data/various_dataSets/cloudcontr/matlab/rosa_skel/vines/';
    subsample_rate = 0.03;
else
    out_path = '/home/fyandun/Documentos/Manipulator/Data/various_dataSets/cloudcontr/matlab/rosa_skel/synthetic/';
    subsample_rate = 0.05;
end

% figure()
% transformed_cloud1 = rotx(90)*P.cpts';
% transformed_cloud1(3,:) = transformed_cloud1(3,:) - min(transformed_cloud1(3,:)); 
% scatter3(transformed_cloud1(1,:),transformed_cloud1(2,:),transformed_cloud1(3,:),30,'.r');

ptCloud = pointCloud(P.cpts);
% ptCloud.Intensity = (1:length(transformed_cloud1))';
ptCloudOut = pcdownsample(ptCloud,'gridAverage', subsample_rate); %0.05 funciono bien para tree_1
pcshow(ptCloudOut)
% scatter3(transformed_cloud1(:,1),transformed_cloud1(:,2),transformed_cloud1(:,3),30,'.r');


save(strcat(out_path, test_file, '_3.mat'), 'ptCloudOut', 'scale', 'xyz_base')

%% step 2: Point to curve �C by cluster ROSA2.0
tic
P.sample_radius = P.diameter*0.02;
P = rosa_lineextract(P,P.sample_radius, 1);
disp(sprintf('to curve:'));
toc
%% show results
figure('Name','Original point cloud and its contraction');movegui('northeast');set(gcf,'color','white')
scatter3(P.pts(:,1),P.pts(:,2),P.pts(:,3),30,'.','MarkerEdgeColor', GS.PC_COLOR);  hold on;
scatter3(P.cpts(:,1),P.cpts(:,2),P.cpts(:,3),30,'.r'); axis off;axis equal;set(gcf,'Renderer','OpenGL');
camorbit(0,0,'camera'); axis vis3d; view(0,90);view3d rot;


figure('Name','Original point cloud and its skeleton'); movegui('center');set(gcf,'color','white');
scatter3(P.pts(:,1),P.pts(:,2),P.pts(:,3),20,'.','MarkerEdgeColor', GS.PC_COLOR);  hold on;
showoptions.sizep=400;showoptions.sizee=2;
plot_skeleton(P.spls, P.spls_adj, showoptions);
axis off;axis equal;set(gcf,'Renderer','OpenGL');view(0,90);view3d rot;
%% save results
default_filename = sprintf('%s_contract_t(%d)_nn(%d)_WL(%f)_WH(%f)_sl(%f)_skeleton.mat',...
    P.filename(1:end-4), t, P.k_knn, initWL, WC, sl);
save(default_filename,'P');