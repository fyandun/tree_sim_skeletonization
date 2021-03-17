function [model, inlierIndices, outlierIndices] = segment_tree(pcCloud, trunk_canopy)
%trunk_canopy = 1 if just want to get the trunk of the tree
%trunk_canopy = 0 if want to get the canopy points
if trunk_canopy
    max_error = 0.1;
else
    max_error = 1.5;
end
%% Fit cylinder using ransac
maxDistance = 0.005;
delta_z = 0.05;

change_error_total = [];

tree_coordinates = pcCloud.Location;

for j=10:100
    
bias_height = delta_z * j;
z_lim = min(tree_coordinates(:,3))+bias_height;

if z_lim > max(tree_coordinates(:,3))   
%     z_lim = max(tree_coordinates(:,3));
  break
end


roi = [min(tree_coordinates(:,1)) max(tree_coordinates(:,1)) min(tree_coordinates(:,2)) max(tree_coordinates(:,2)) min(tree_coordinates(:,3)) z_lim];
sampleIndices = findPointsInROI(pcCloud,roi);

pc = select(pcCloud,sampleIndices);
% figure
% pcshow(pc)
% title('Cylinder Point Cloud')

referenceVector = [0,0,1];

[model, inlierIndices, outlierIndices, mean_error_inl_out] = pcfitcylinder_fy(pc,maxDistance,...
        referenceVector);
    
% mean_error_inl = mean_error(1);
% mean_error_outl = mean_error(2);
% 
if j>10    
    change_error_total = [change_error_total mean_error_inl_out(2)];
    if mean_error_inl_out(2) > max_error
       break 
    end  
end
% plot(model)  
end
% figure
% plot(change_error_total)

