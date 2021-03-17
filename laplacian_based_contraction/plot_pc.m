%%
% test_file = 'mega_cloud1_lab';
test_file = 'mega_cloud4';
filename = strcat('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/', test_file, '.pcd');
tree = pcread(filename);
tfmat = eye(4);
k = strfind(test_file, 'lab');
if isempty(k)
    tfmat(1:3,1:3) = roty(-90);
else
    tfmat(1:3,1:3) = rotx(90);
end
tform = affine3d(tfmat);
figure
hold on
ptcloud1 = pctransform(tree, tform);
scatter3(ptcloud1.Location(:,1), ptcloud1.Location(:,2), ptcloud1.Location(:,3),1, '.k')
% pcshow(ptcloud1)
axis equal
% set(gca, 'color', [1 1 1])
% set(gcf, 'color', [1 1 1])

%%
out_path_csv = strcat('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/cloudcontr/matlab/point_clouds/csv/', test_file, '_original.csv');
csvwrite(out_path_csv, ptcloud1.Location)


%%

shp = alphaShape(double(ptcloud1.Location(:,1)), double(ptcloud1.Location(:,2)), double(ptcloud1.Location(:,3)));

shp.Alpha = 0.01;
plot(shp)
[bf, P] = boundaryFacets(shp);
out_path_stl = strcat('/home/fyandun/Documentos/Manipulator/Data/various_dataSets/cloudcontr/matlab/point_clouds/stl/', test_file, '_original.stl');
stlwrite(triangulation(bf,P), out_path_stl)