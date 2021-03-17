
%% save the bud locations (some unknown corrdinate frame - requires manual alignment)
clear
load('bud_locations_lab.mat')
ptCloud = pointCloud(single(buds));
pcwrite(ptCloud,'/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/bud_locations_lab.pcd', 'Encoding','binary')

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



%% save initial tag locations (wrt base of the point cloud)
save('tag_init_pos.mat', 'init_pos_all')
ptCloud_tags = pointCloud(single(init_pos_all(1:3,:)'));
pcwrite(ptCloud_tags,'/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/tag_init_pos.pcd', 'Encoding','binary')

%% read contact points (obtained by hand) and initial marker points 
contact_points = csvread('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/contact_points/contact_points_all.csv');
p_cp = plot3(contact_points(:,1), contact_points(:,2), contact_points(:,3), 'xr');

tags_init_pos = csvread('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/tags_positions_corrected/tag_init_pos_corrected_model_all.csv');
p_tag_init = plot3(tags_init_pos(:,1), tags_init_pos(:,2), tags_init_pos(:,3), 'sg');

legend ([p_cp p_tag_init], 'contact points', 'markers init pos')
view(0,0)
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

%% find the tform to get the exact points in my skeleton 
tform_skel_tags = cell(length(ids_all),1);
for i =1:length(ids_all)
    index = markers_branch_index(i);
    if ~isnan(index)
        tform_skel_tags{i} =  init_pos_all(1:3,i)' - a_initial.branches(index).branch_position;
        plot_markers_model = plot3(a_initial.branches(index).branch_position(1), a_initial.branches(index).branch_position(2), a_initial.branches(index).branch_position(3), 'or');
    else
        tform_skel_tags{i} = nan;
    end
end
legend([plot_markers_model], 'Markers init pos')
%% steady state tests
bag_path = '/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/lab_vine/bagFiles/';
type_test = 'steady';
axis = 'y';
marker = '1'; 
test_n = '3';
filename = strcat(bag_path, axis, '_', marker, '_', test_n, '_', type_test, '.bag');
steady_positions_ = read_steady(filename, Tform_to_base); %wrt to base of point_cloud (tag 5 and base of point cloud are off by 6cm)

%account for both tfs, the first to the rotated one tag_init_tform and the traslation
%error tform_skel_tags
steady_positions = appply_tfs(steady_positions_, tag_init_tform, tform_skel_tags);

for i=1:length(steady_positions)  
    %calculate the relative displacement only - remember that I manually
    %moved some tag initial locations, so it is not possible to plot these final
    %positions as they are off
    if ~isempty(steady_positions(i).position)
        steady_positions(i).rel_displacement = steady_positions(i).position - init_positions(i).position(1:3);
        plot_final_pos_steady = plot3(steady_positions(i).position(1), steady_positions(i).position(2), steady_positions(i).position(3), '<m');
    end
end
legend(plot_final_pos_steady, 'Final Pos GT')

%% save the steady poistions - debugging
steady_positions_all = [];
for i=1:length(steady_positions_)  
    if ~isempty(steady_positions_(i).position)
        data = tag_init_tform*[steady_positions_(i).position;1];
        steady_positions_all = [steady_positions_all data(1:3)];
    end
end
ptCloud = pointCloud(single(steady_positions_all'));
filename_ptCloud_steady = strcat(bag_path, 'points/', axis, '_', marker, '_', test_n, '_', type_test, '.pcd');
pcwrite(ptCloud,filename_ptCloud_steady, 'Encoding','binary')
%% simulate the dynamics
aux = find (contact_point_lab_ids == str2double(marker));
filename_forces = strcat(bag_path, 'forces/', axis, '_', marker, '_', test_n, '_', type_test, '.txt');
force_data = importdata(filename_forces);

dt = 0.01;
tFinal = 2.5;
itFinal = tFinal/dt;
t = 0;

branch_id = contact_point_branch_id(aux);
wrench.branch_id = find(a.indices_ids == branch_id);
wrench.f_ext = force_data';
% wrench.f_ext = [0 0 0]';

wrench.t_ext = [0 0 0]';
% a.expand_dynamics([])
a.expand_dynamics(wrench)
for it=1:itFinal
    t = t + dt*it;
    
%     if it == itFinal
%         aux=1;
%     end
    a.dynSim(t)
    a.expand_dynamics(wrench)
    
%      a.draw  
    
    angles_per_branch = [];
    for j=2:length(a.branches)
       angles_per_branch = [angles_per_branch a.branches(j).theta_all(:,it)];        
    end
%     pause(0.001)
    
end

%% plot the final positions of my model
for i=1:length(a.branches)
   plot3(a.branches(i).branch_position(1), a.branches(i).branch_position(2), a.branches(i).branch_position(3), 'Color', [0.9290 0.6940 0.1250], 'Marker', 'x') 
end
legend([plot_final_pos_steady plot_markers_model], 'Final Pos GT', 'Markers init pos')
%% calculate the displacement of the model 
final_pos_model = struct([]);
for j = 1:length(markers_branch_id)
    index = markers_branch_index(j);
    final_pos_model.tag_id = ids_all(j);
    final_pos_model.displacement = a.branches(index).branch_position - a_initial.branches(index).branch_position;    
end


%% transcient tests
type_test = 'transcient';
axis = 'x';
marker = '15'; %made a mistake 15 -> marker 14, 16->marker15, 13->marker12 
test_n = '1';
filename = strcat(bag_path, axis, '_', marker, '_', test_n, '_', type_test, '.bag');
transcient_data = read_transcient(filename, Tform_to_base); %wrt to base of point_cloud (tag 5 and base of point cloud are off by 6cm)
