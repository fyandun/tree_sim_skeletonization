%%
clear
test_name = 'mega_cloud16';

path_ = '/home/fyandun/Documentos/Manipulator/Data/various_dataSets/cloudcontr/matlab/vines_point_clouds_processed/pcd(Haussdorf_dist)/';
filename = strcat(path_, test_name, '_Hausdorff.csv');
tree = dlmread(filename);


fig = figure;
ptCloud = pointCloud(tree(:,1:3), 'Intensity', tree(:,4));
pcshow(ptCloud)
a = colorbar;
a.Color = [0 0 0];
% a.Label.String = 'Hausdorff Dist to Generated Model';
% a.Label.FontSize = 18;
xlabel('x(m)')
ylabel('y(m)')
zlabel('z(m)')
set(findall(gcf,'-property','FontSize'),'FontSize',22)


color_ = [255, 255, 255]/255;
set(gca,'color',color_);
set(gcf,'color',color_);
set(gca, 'XColor', [0. 0. 0.], 'YColor', [0 0. 0.], 'ZColor', [0. 0. 0.])
axis tight
view(125,35)

%%
path_fig_out = strcat(path_, 'images/',test_name, '_v1.svg');
fig.InvertHardcopy = 'off';
saveas(gcf, path_fig_out)

%% Graphical boxplot por frame - real vines
path_ = '/home/fyandun/Documentos/Manipulator/Data/various_dataSets/cloudcontr/matlab/vines_point_clouds_processed/pcd(Haussdorf_dist)/';

test_name_all = {'mega_cloud', 'mega_cloud2', 'mega_cloud3', 'mega_cloud4', 'mega_cloud5', 'mega_cloud6', 'mega_cloud7', 'mega_cloud8', 'mega_cloud9', 'mega_cloud10', ...
    'mega_cloud13', 'mega_cloud14', 'mega_cloud15', 'mega_cloud16'};
tamanos = zeros(1, length(test_name_all));
Hausdorff_distances = [];
for i=1:length(test_name_all)
    filename_all = strcat(path_, test_name_all{i}, '_Hausdorff.csv');
    tree_generated = dlmread(filename_all);
    Hausdorff_distances = [Hausdorff_distances; tree_generated(:,4)];
    tamanos(i) = size(tree_generated,1); 
end

nombre_toma1 = repmat({'Vine 1'}, 1, tamanos(1));
nombre_toma2 = repmat({'Vine 2'}, 1, tamanos(2));
nombre_toma3 = repmat({'Vine 3'}, 1, tamanos(3));
nombre_toma4 = repmat({'Vine 4'}, 1, tamanos(4));
nombre_toma5 = repmat({'Vine 5'}, 1, tamanos(5));
nombre_toma6 = repmat({'Vine 6'}, 1, tamanos(6));
nombre_toma7 = repmat({'Vine 7'}, 1, tamanos(7));
nombre_toma8 = repmat({'Vine 8'}, 1, tamanos(8));
nombre_toma9 = repmat({'Vine 9'}, 1, tamanos(9));
nombre_toma10 = repmat({'Vine 10'}, 1, tamanos(10));
nombre_toma11 = repmat({'Vine 11'}, 1, tamanos(11));
nombre_toma12 = repmat({'Vine 12'}, 1, tamanos(12));
nombre_toma13 = repmat({'Vine 13'}, 1, tamanos(13));
nombre_toma14 = repmat({'Vine 14'}, 1, tamanos(14));

Nombres = [nombre_toma1 nombre_toma2 nombre_toma3 nombre_toma4 nombre_toma5 nombre_toma6 nombre_toma7 nombre_toma8 nombre_toma9 nombre_toma10 nombre_toma11 nombre_toma12 nombre_toma13 nombre_toma14];

fig = figure();
% DatosBoxPlot = boxplot(Hausdorff_distances, Nombres, 'whisker', 2.5);
DatosBoxPlot = boxplot(Hausdorff_distances, Nombres, 'DataLim', [-Inf 0.04]);
set(gca, 'FontSize',15)
title('Hausdorff Distance per Tree', 'FontSize',20);
ylabel('Hausdorff dist to ground truth (m)', 'FontSize',20);
xlhand = get(gca,'xlabel');
set(xlhand, 'FontSize',20)

