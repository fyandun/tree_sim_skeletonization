%%
clear
close all
% tic
% s = rng
% load('random_seed.mat', 's')
% rng(s)
test_file = 'mega_cloud1_lab_3';
% test_file = 'mega_cloud16';
path = strcat('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/cloudcontr/matlab/rosa_skel/vines/', test_file , '.mat'); 
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
a = tree_v1(points);

%%
%OJO QUE ANTES DE EXPAND_DYNAMICS SE DEBE CRECER Y DIBUJAR PARA QUE SE
%ACTUALICE EL ARBOL CON EL CAMBIO EN LAS DIMENSIONES DE LOS CILINDROS
tic
for i=1:1
    
    
   a.grow 
end
toc

% %debugging
% a.draw_label
% a.draw


    
a.draw_cylinders
tic
a.expand_dynamics([])
toc
% toc
a.draw

a.draw_label
a_initial = a;

%% load the tag locations
bag_path = '/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/lab_vine/bagFiles/';

ids_all = [1 3 4 5 6 12 13 14 15];
tag_init_tform = importdata('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/first_tform_tags.txt');
init_pos_all = [];

% figure
% hold on
[init_positions, Tform_to_base] = read_init_conds(bag_path); %wrt to base of point_cloud (tag 5 and base of point cloud are off by 6cm)
for i=1:length(init_positions)  
if i == 4
    color = 'ok';
    aux = init_positions(i).position + [0 0 -0.06 0]';
    aux = tag_init_tform * aux;
    
    init_pos_all = [init_pos_all aux];
else
    color = 'xr';
    init_pos_all = [init_pos_all tag_init_tform*init_positions(i).position];
end
% plot3(init_pos_all(1,i), init_pos_all(2,i), init_pos_all(3,i), color)
end
% view(90,0)


%% these variables allow to simulate the dynamics
%this is for simulate the dynamics
contact_point_lab_ids = [15 14 3 1 12 4];
contact_point_branch_id = [141 179 49 105 32 232];
contact_point_branch_index = [240 90 270 96 258 222];

%this is for evaluate the displacements, the equivalent of the branch id to
%the tag positions
ids_all = [1 3 4 5 6 12 13 14 15];
markers_branch_id = [177 24 241 nan 76 nan 91 261 106]; %tag 12 placed outside fov of camera - nan
markers_branch_index = [65 281 120 nan 245 nan 180 75 267]; %tag 12 placed outside fov of camera - nan


%% transcient tests
transcient_test = 1;
bag_path = '/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/lab_vine/bagFiles/';
type_test = 'steady';
test_axis = 'x';
marker = '3'; 
test_n = '1';

if transcient_test
    type_test = 'transcient';
end
save_video = 1;
if save_video 
    videoOutPath = '/home/fyandun/Documentos/Manipulator/papers/cvpr2020/presentation/images/';

    video_file = strcat(videoOutPath,test_axis, '_', marker, '_', test_n, '_', type_test, '_sim.avi');
    v = VideoWriter(video_file);
    v.FrameRate = 30;
    open(v)
    
end
%% simulate the dynamics
aux = find (contact_point_lab_ids == str2double(marker));
filename_forces = strcat(bag_path, 'forces/', test_axis, '_', marker, '_', test_n, '_', type_test, '.txt');
force_data = importdata(filename_forces);

dt = 0.01;
tFinal = 5; %2.5
itFinal = tFinal/dt;
t = 0;

branch_id = contact_point_branch_id(aux);
wrench.branch_id = find(a.indices_ids == branch_id);
%wrench.f_ext = force_data';
wrench.f_ext = [0 0 0]';

wrench.t_ext = [0 0 0]';
% a.expand_dynamics([])
a.expand_dynamics(wrench)
t_total = [];
for it=1:itFinal
    t = t + dt;
    t_total = [t_total t];
%     if it == itFinal
%         aux=1;
%     end
    a.dynSim(t)
    a.expand_dynamics(wrench)
    
    if save_video
        a.draw_cylinders
        view(7,4)
        frame = getframe(gcf);
        writeVideo(v,frame);
        pause(0.01)
    end
    
    
    angles_per_branch = [];
    for j=2:length(a.branches)
       angles_per_branch = [angles_per_branch a.branches(j).theta_all(:,it)];        
    end
    
%     a.draw_cylinders
%     view(0,0) % view(7,4)
%     pause(0.01)
    
end


%     a.change_init_conditions;

if save_video
     close(v)
end


%%
branch = wrench.branch_id;
% branch = 2;
figure
subplot(3,1,1)
plot(t_total, a.branches(branch).theta_all(1,:)*180/pi, '.-')
title('\theta x response')
xlabel('t(s)')
ylabel('x(m)')

subplot(3,1,2) 
plot(t_total, a.branches(branch).theta_all(2,:)*180/pi, '.-')
title('\theta y response')
xlabel('t(s)')
ylabel('y(m)')

subplot(3,1,3) 
plot(t_total, a.branches(branch).theta_all(3,:)*180/pi, '.-')
title('\theta z response')
xlabel('t(s)')
ylabel('z(m)')



%% 