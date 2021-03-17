%%
% leaves_positions = [];
% save('test.mat', 'leaves_positions')
clear
test_case = 2;
file_path = strcat('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/trees/tree_', num2str(test_case), '.ply');

fig=figure;
hold on
axis tight
tree = pcread(file_path);
tree_coordinates = tree.Location;
tree_color = tree.Color;
% tfmat = eye(4);
% tfmat(1:3,1:3) = roty(-90);
% tform = affine3d(tfmat);
% ptcloud1 = pctransform(tree, tform);
% pcshow(tree)
scatter3(tree_coordinates(:,1), tree_coordinates(:,2), tree_coordinates(:,3),0.1, tree_color./255, '.', 'Tag', 'pcviewer');

%%
aux1 =  find(mean(tree_color,2) < 50);
strcuture_tree = tree_coordinates(aux1,:);
structure_color = tree_color(aux1,:);
figure()
scatter3(strcuture_tree(:,1), strcuture_tree(:,2), strcuture_tree(:,3),0.1, structure_color./255, '.', 'Tag', 'pcviewer');


%% Fit cylinder using ransac
%get the trunk model
[model, ~, ~] = segment_tree(tree, 1);

aux = find(tree_coordinates(:,3)>min(tree_coordinates(:,3))+model.Height);
potential_canopy = tree_coordinates(aux,:);
potential_color_canopy = tree_color(aux,:);

% cova = cov(potential_canopy);
% media = mean(potential_canopy);
% diff_data = potential_canopy - media;
% mahal_dist = sum(diff_data/cova.*diff_data,2);

% aux = find(tree_coordinates(:,3)>min(tree_coordinates(:,3))+model.Height);
% canopy = tree_coordinates(aux,:);
% color_canopy = tree_color(aux,:);

fig1 = figure;
scatter3(potential_canopy(:,1), potential_canopy(:,2), potential_canopy(:,3),0.1, potential_color_canopy./255, '.', 'Tag', 'pcviewer')

dcm_obj = datacursormode(fig1);
set(dcm_obj,'UpdateFcn',{@myupdatefcn2,mean(potential_color_canopy,2)}) 

%%
aux1 =  find(mean(potential_color_canopy,2) > 150);
canopy = potential_canopy(aux1,:);
color_can = potential_color_canopy(aux1,:);

shp = alphaShape(canopy(:,1),canopy(:,2),canopy(:,3),0.5);

figure;
scatter3(canopy(:,1), canopy(:,2), canopy(:,3),0.1, color_can./255, '.', 'Tag', 'pcviewer')
plot(shp)

%% rejection sampling 1
radius_q = (max(canopy(:,1)) - min(canopy(:,1)))/2;
center_q = mean(canopy);
n_samples = 1000;
sampled_points = zeros(n_samples,3);
count = 0;

while count < n_samples
    psi = 2*pi*rand;
    theta = 2*pi*rand;
    r = radius_q*rand; 
    
    x = center_q(1) + r*sin(theta)*cos(psi);
    y = center_q(2) + r*sin(theta)*sin(psi);
    z = center_q(3) + r*cos(theta);
    
    if inShape(shp, x,y,z)
        count = count + 1;
        sampled_points(count, :) = [x y z]; 
    end        
end
plot3(sampled_points(:,1),sampled_points(:,2), sampled_points(:,3), 'xr')

leaves_pos = sampled_points;
model_center_z = model.Center(3)-model.Height/2;
pivot_z = model.Center(3)+model.Height/2;
root_pos.Position = [model.Center(1), model.Center(2), model_center_z];
pivot = [model.Center(1), model.Center(2), pivot_z];
save('root_leaves_tree_rej_samp_1.mat','leaves_pos','pivot', 'root_pos')

%% rejection sampling 2
psi_total = 2*pi*rand(1,n_samples);
theta_total = 2*pi*rand(1,n_samples);
sampled_points = zeros(n_samples,3);
n_samples = 100;
count = 0;
while count < n_samples
    psi = psi_total(count+1);
    theta = theta_total(count+1);
    r = radius_q*rand; 
    
    x = center_q(1) + r*sin(theta)*cos(psi);
    y = center_q(2) + r*sin(theta)*sin(psi);
    z = center_q(3) + r*cos(theta);
    
    if inShape(shp, x,y,z)
        count = count + 1;
        sampled_points(count, :) = [x y z]; 
    end        
end

plot3(sampled_points(:,1),sampled_points(:,2), sampled_points(:,3), 'xr')
leaves_pos = sampled_points;
model_center_z = model.Center(3)-model.Height/2;
pivot_z = model.Center(3)+model.Height/2;
root_pos.Position = [model.Center(1), model.Center(2), model_center_z];
pivot = [model.Center(1), model.Center(2), pivot_z];
save('root_leaves_tree_rej_samp_2.mat','leaves_pos','pivot', 'root_pos')

%% Just subsampling
n_points = length(canopy);
objt_points = 50;
percentage = 100*objt_points/n_points;
ptCloudOut = pcdownsample(pointCloud(canopy),'gridAverage',0.5);
figure()
pcshow(ptCloudOut)

leaves_pos = ptCloudOut.Location;
model_center_z = model.Center(3)-model.Height/2;
pivot_z = model.Center(3)+model.Height/2;
root_pos.Position = [model.Center(1), model.Center(2), model_center_z];
pivot = [model.Center(1), model.Center(2), pivot_z];
save('root_leaves_tree_subs.mat','leaves_pos','pivot', 'root_pos')






