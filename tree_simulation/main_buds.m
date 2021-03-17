%%
clear
close all
% tic
% s = rng
% load('random_seed.mat', 's')
% rng(s)
test_file = '4';
folder = '/home/fyandun/Documentos/Manipulator/Data/various_dataSets/real_data/vines/new_experiments/';
path_buds = strcat(folder, 'mega_buds_', test_file , '.pcd'); 
buds_data = pcread(path_buds);
path_cloud = strcat(folder, 'mega_cloud_', test_file , '.pcd'); 
complete_cloud = pcread(path_cloud);

%%%%% pre process points
points = buds_data.Location;
points = roty(90)*points';
points = points';
cloud = complete_cloud.Location;
cloud = roty(90)*cloud';
cloud = cloud';
points = points - mean(points);
cloud = cloud - mean(cloud);

figure()
plot3(cloud(:,1), cloud(:,2), cloud(:,3), '.b')
hold on
plot3(points(:,1), points(:,2), points(:,3), 'xr')

xyz_base = [0 0 0]; %for some reason this value is wrong from the LBC
[~, I] = min(points(:,3));
aux = points(I,:);
bias_base_xy = aux - xyz_base;
% points = points - [bias_base_xy(1:2) 0];
points = points - bias_base_xy;

figure()
% plot3(cloud(:,1), cloud(:,2), cloud(:,3), '.b')
% hold on
plot3(points(:,1), points(:,2), points(:,3), 'xr')

a = tree_v2(double(points));

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


    
a.draw_cylinders
tic
a.expand_dynamics([])
toc
% toc
a.draw

a.draw_label
a_initial = a;

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

dt = 0.05;
tFinal = 5;
itFinal = tFinal/dt;
t = 0;

branch_id = 106; %30; 141;
wrench.branch_id = find(a.indices_ids == branch_id); %241
wrench.f_ext = [0 0 -95.6370]';
% wrench.f_ext = [0 0 0]';

wrench.t_ext = [0 0 0]';
% a.expand_dynamics([])
% a.expand_dynamics(wrench)
timer = tic;
for it=1:itFinal
    t = t + dt;
    
%     if t == 2.5
%         aux=1;
%     end
    tic
    %a.expand_dynamics(wrench)
    %a.dynSim(t)    
    pruned_tree.expand_dynamics(wrench)
    pruned_tree.dynSim(t)
    toc
%      a.draw  
    
%     angles_per_branch = [];
%     for j=2:length(a.branches)
%        angles_per_branch = [angles_per_branch a.branches(j).theta_all(:,it)];        
%     end
%     a.draw_cylinders
% %     view(17,24)
%     view(-160,20)
%     axis equal
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
xlabel('x(m)')

ylabel('y(m)')
zlabel('z(m)')

subplot(3,1,2) 
plot(a.branches(branch).theta_all(2,:)*180/pi, '.-')
title('\theta x response')
xlabel('x(m)')
ylabel('y(m)')
zlabel('z(m)')

subplot(3,1,3) 
plot(a.branches(branch).theta_all(3,:)*180/pi, '.-')
title('\theta x response')
xlabel('x(m)')
ylabel('y(m)')
zlabel('z(m)')
%%
a.change_init_conditions;
%%

%%%%ES COMO QUE LAS ORIENTACIONES SE ESTAN ACUMULANDO!
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
%%
% todos_branches = cell(length(a.branches));
% for j=2:length(a.branches)
%    for k=1:itFinal
%        z = a.branches(j).theta_all(1,k)*180/pi;
%        y = a.branches(j).theta_all(2,k)*180/pi;
%        x = a.branches(j).theta_all(3,k)*180/pi;
%        
%        if z > 1e-4 || y > 1e-4 || x > 1e-4
%            aux = 1;
%        end                    
%    end
%     
% end

% %%
% obj = a.branches(2,1);
% dt = 0.01;
% tFinal = 10;
% itFinal = tFinal/dt;
% t = 0;
% theta_total = zeros(3,itFinal);
% obj.beta = 1;
% for it=1:itFinal
%     t = t + dt*it;
%     
%     Rjw = inv(obj.orientation);            
%     p = obj.op_hat - obj.branch_parent.branch_position;
%     aux = (Rjw*p');
%     I_tilde = Rjw*obj.I_p_w_hat*Rjw' + obj.mass_p_hat*skewm(aux)'*skewm(aux);
% 
%     torque_tilde = Rjw*obj.torque_hat;
% 
%     L = chol(I_tilde); 
%     [x,lambda] = eig(inv(L')*diag(obj.K)*inv(L));
%     U = L\x;
%     delta = U'*I_tilde*U;
% 
%     net_ext_torque = U'*torque_tilde;
%     B = diag(obj.beta*lambda + obj.alpha*delta);
%     lambda_diag = diag(lambda);
%     theta_ = zeros(3,1);
%     %state here the initial conditions
%     
%     for k=1:3
%         
%         init = obj.init_angles(k);
%         init_prime = obj.init_angles_prime(k);        
%         
%         k_spring = lambda_diag(k);
%         b = B(k);
%         r = 0.5*[-b+sqrt(b^2-4*k_spring) -b-sqrt(b^2-4*k_spring)];
%         y_part = net_ext_torque(k)/k_spring;
%         coefs = [0 0];
%         if isreal(r)
%             if r(1) ~=r(2)
% 
%                 A_coefs = [1 1;r(1) r(2)];
%                 B_coefs = [init - y_part; init_prime];
%                 coefs = A_coefs\B_coefs; 
% 
%                 theta_(k) = coefs(1) * exp(r(1)*t) + coefs(2) *exp(r(2)*t) + y_part;
%             else
%                 coefs(1) = init -y_part;
%                 coefs(2) = init_prime - coefs(1)*r(1);
%                 theta_(k) = (coefs(1) + coefs(2)*t)*exp(r(1)*t) + y_part; 
%             end
%         else
%             real_part = real(r(1));
%             imag_part = imag(r(1));
%             coefs(1) = init + y_part;
%             coefs(2) = (init_prime -real_part*coefs(1))/imag_part; 
%             theta_(k) = wrapToPi(exp(t*real_part)*( coefs(1)*cos(imag_part*t) + coefs(2)*sin(imag_part*t) ) + y_part);
%         end
%     end
%     theta = U*theta_;
%     theta_total(:,it) = theta; 
%     
% 
% end

%%
% count = 0;
% v = VideoReader('/home/fyandun/Vídeos/dyn_sim.mp4');
% 
% v_write = VideoWriter('/home/fyandun/Vídeos/dyn_sim_fast.mp4');
% v_write.FrameRate= 60;
% open(v_write);
% while hasFrame(v)
%     count = count +1;
%     frame = readFrame(v);
%     
%     if count >10
%         writeVideo(v_write,frame);
%     end
% end
% 
% close(v_write);
