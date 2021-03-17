%%
clear
close all
% tic
% s = rng
% load('random_seed.mat', 's')
% rng(s)
% test_file = 'mega_cloud1_lab_3';
% test_file = 'mega_cloud16';
% addpath(genpath('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/cloudcontr/matlab/tree_vines/build_sdf'))


% test_file = 'mega_cloud_lab_new_2'; 
test_file = 'mega_cloud_lab_complex_no_holes_4';
path = strcat('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/cloudcontr/matlab/rosa_skel/vines/', test_file , '.mat'); 
% test_file = 'mega_cloud_4';
% path = strcat('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/cloudcontr/matlab/rosa_skel/vines/field/', test_file , '.mat');

load(path)
%scale = 0.384307830170736;
% xyz_base = [-0.00142602920941766,-0.184963564065752,0.218892573085382];
points = ptCloudOut.Location/scale;

xyz_base = [0 0 0]; %for some reason this value is wrong from the LBC
[~, I] = min(points(:,3));      
aux = points(I,:);
bias_base_xy = aux - xyz_base;
% points = points - [bias_base_xy(1:2) 0];
points = points - bias_base_xy;

%points(1:10,:) = []; %debugging
points = points*rotz(180); % just for mega_cloud_lab_new_2
% points = points*rotz(90); % just for mega_cloud16
a = tree_v1(points);

%%
%OJO QUE ANTES DE EXPAND_DYNAMICS SE DEBE CRECER Y DIBUJAR PARA QUE SE
%ACTUALICE EL ARBOL CON EL CAMBIO EN LAS DIMENSIONES DE LOS CILINDROS
tic
for i=1:1
    
    
   a.grow 
   a.copy_branches
end
toc

% %debugging
% a.draw_label
% a.draw


    
a.draw_cylinders(0) %0 means is the original tree, without fusing similar links
tic
a.expand_dynamics([])
toc
% toc
a.draw

a.draw_label
% a_initial = a;


%% this section is only useful to evaluate the dynamic model
dynamics_eval_v1

%% save the tree object for evaluation
path_save_obj_ = '/home/fyandun/Documentos/Manipulator/Data/various_dataSets/cloudcontr/matlab/vines_objects_built/';
path_save = strcat(path_save_obj_, test_case, '.mat');
save(path_save, 'a', 'a_initial')

%% generate a point cloud and from the created structure - to compare with the original 

points = convert_to_ptcloud(a, 200);
path_csv = strcat('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/cloudcontr/matlab/vines_point_clouds_processed/csv/', test_file, '_processed.csv');
csvwrite(path_csv, points)


shp = alphaShape(points(:,1), points(:,2), points(:,3));

shp.Alpha = 0.01;
plot(shp)
[bf, P] = boundaryFacets(shp);
path_stl = strcat('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/cloudcontr/matlab/vines_point_clouds_processed/stl/', test_file, '_processed.stl');
stlwrite(triangulation(bf,P), path_stl)
%%

dt = 0.01;
tFinal = 5;
itFinal = tFinal/dt;
t = 0;

branch_id = 363; %30; 141;
% wrench.branch_id = find(a.indices_ids == branch_id); %241
wrench.branch_id = find(pruned_tree.indices_ids == branch_id); %241
% wrench.f_ext = [0 0 0]';
% wrench.f_ext = [3.039 -0.294184 -49.5694]';
wrench.f_ext = [0 0 -95.6370]'; %for the original model

wrench.t_ext = [0 0 0]';
% a.expand_dynamics([])
% a.expand_dynamics(wrench)
timer = tic;
for it=1:itFinal
    t = t + dt;
    
%     if t == 2.5
%         aux=1;
%     end
%     tic
%     a.expand_dynamics(wrench)
%     a.dynSim(t)    
    pruned_tree.expand_dynamics(wrench)
    pruned_tree.dynSim(t)
%     toc
%      a.draw  
    
%     angles_per_branch = [];
%     for j=2:length(a.branches)
%        angles_per_branch = [angles_per_branch a.branches(j).theta_all(:,it)];        
%     end
%     a.draw_cylinders
%     pruned_tree.draw_cylinders
% % %     view(17,24)
%      view(-160,20)
%      axis equal
%     pause(0.01)
    
end

toc(timer)

%%
branch = wrench.branch_id;
% branch = 2;
figure
subplot(3,1,1)
plot(a.branches(branch).theta_all(1,:)*180/pi, '.-')
title('\theta x response')
xlabel('sample')
ylabel('angle(deg)')

subplot(3,1,2) 
plot(a.branches(branch).theta_all(2,:)*180/pi, '.-')
title('\theta y response')
xlabel('sample')
ylabel('angle(deg)')

subplot(3,1,3) 
plot(a.branches(branch).theta_all(3,:)*180/pi, '.-')
title('\theta z response')
xlabel('sample')
ylabel('angle(deg)')
%%
a.change_init_conditions;
%%

%%%%
figure
subplot(3,1,1)
plot(angles_per_branch(1,:)*180/pi, '.')
subplot(3,1,2) 
plot(angles_per_branch(2,:)*180/pi, '.')
subplot(3,1,3) 
plot(angles_per_branch(3,:)*180/pi, '.')

%%

tree = pcread('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/mega_cloud16.pcd');
% pcshow(tree)
tform_ = eye(4)
tform_(1:3,1:3) = roty(-90);
tform = affine3d(tform_)
ptcloud1 = pctransform(tree, tform);
pcshow(ptcloud1)

