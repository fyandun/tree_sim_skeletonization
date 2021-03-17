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
transcient_test = 1;
bag_path = '/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/lab_vine/bagFiles/';
type_test = 'steady';
test_axis = 'x';
marker = '3'; 
test_n = '1';
filename = strcat(bag_path, test_axis, '_', marker, '_', test_n, '_', type_test, '.bag');
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

%% transcient tests
if transcient_test
    type_test = 'transcient';
end
save_video = 0;
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
wrench.f_ext = force_data';
%wrench.f_ext = [0 0 0]';

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
%     view(7,4)
%     pause(0.01)
    
end

if transcient_test
%     a.change_init_conditions;
end
if save_video
     close(v)
end

%% plot the final positions of my model
for i=1:length(a.branches)
   plot3(a.branches(i).branch_position(1), a.branches(i).branch_position(2), a.branches(i).branch_position(3), 'Color', [0.9290 0.6940 0.1250], 'Marker', 'x') 
end
legend([plot_final_pos_steady plot_markers_model], 'Final Pos GT', 'Markers init pos')

%%
branch = wrench.branch_id;
% branch = 2;
figure
subplot(3,1,1)
plot(t_total,a.branches(branch).theta_all(1,:)*180/pi, '.-')
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

%% calculate the displacement of the model 
diag_length = norm(min(points) - max(points));

final_pos_model = struct([]);
all_mse = [];
for j = 1:length(steady_positions)
    index = steady_positions(j).Id;
    if ~isnan(steady_positions(j).position)
        aux = find(ids_all == index);
        displ_tag_model = steady_positions(j).position - a.branches(markers_branch_index(aux)).branch_position;
        all_mse = [all_mse norm(displ_tag_model)];        
%         plot3(steady_positions(j).position(1), steady_positions(j).position(2), steady_positions(j).position(3), '<r')
%         plot3(a.branches(markers_branch_index(aux)).branch_position(1), a.branches(markers_branch_index(aux)).branch_position(2), a.branches(markers_branch_index(aux)).branch_position(3), 'xr')
    end    
end
mse = mean(all_mse);

% %%
% load('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/cloudcontr/matlab/tree_vines/summary_dynamics.mat')
% names = {'x', 'y', 'z'};
% figure()
% hold on
% plot(summary_dynamics_right(1,:), '<g')
% plot(summary_dynamics_left(2,:), 'sb')
% set(gca,'xtick',[1:5],'xticklabel',names)
% xlabel('Axis of motion')
% ylabel('MSE (m)')

%% 