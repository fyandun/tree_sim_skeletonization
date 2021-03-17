%%
mass = 1;
length = 1;
radius = 0.1;


Ix = (1/12)*mass*(3*radius^2+length^2)

Iz = 0.5*mass*radius^2


%%
kp = [20106.1929829747];
kd = kp*0.1;
mass = 1;
dt= 0.001;

r = 2*kd*sqrt(mass*kp);
hk = dt*kp;
    

erp = hk/(hk+r)
cfm = 1/(hk+r)


%%
kp = [20106.1929829747]; %[7733151.14729795] [20106.1929829747]
kd = kp*0.1;
mass = 1;
dt= 0.001;

erp = dt * kp/(dt*kp+kd)
cfm = 1/(dt*kp + kd)
