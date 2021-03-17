%%
r = 2.5;
[X_,Y_,Z_] = sphere;
X = X_*r;
Y = Y_*r;
Z = Z_*r;
surf(X,Y,Z)
axis equal
hold on

%%
x_all = [];
y_all = [];
z_all = [];
r_min = 1;%r*0.9;
r_max = 1;%r*1.1;
for i=1:10000
   u = rand;
   v = rand;
   r_ = r_min + (r_max - r_min)*rand;
   theta = 2*pi*u;
   phi = acos(2*v-1);
   
   x = r_*sin(theta)*cos(phi);
   y = r_*sin(theta)*sin(phi);
   z = r_*cos(theta); 
   
   
   x_all = [x_all;x];
   y_all = [y_all;y];
   z_all = [z_all;z];
end
plot3(x_all, y_all, z_all, 'xr')
xlabel('x')
ylabel('y')
zlabel('z')


%%
x_all = [];
y_all = [];
z_all = [];
r_min = 2.5;%r*0.9;
r_max = 2.5;%r*1.1;
for i=1:1000
   u = -1 + 2*rand;
   r_ = r_min + (r_max - r_min)*rand;
   theta = pi*rand;
   
   x = r_*sqrt(1-u*u)*cos(theta);
   y = r_*sqrt(1-u*u)*sin(theta);
   z = r_*u; 
   
   
   x_all = [x_all;x];
   y_all = [y_all;y];
   z_all = [z_all;z];
end
plot3(x_all, y_all, z_all, 'xr')
xlabel('x')
ylabel('y')
zlabel('z')