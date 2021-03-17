%%
[X,Y,Z] = sphere;
r = 5;
X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;

surf(X2,Y2,Z2)

%%
r = 5;
point = [];
for i= 1:2
    d = 100;
    y = -100;
    while d >= 1 || y < 0
        x1 = -1 + (1+1)*rand()
        x2 = -1 + (1+1)*rand()
        d = x1^2 + x2^2;
        y = 2*x2*sqrt(1 - x1^2 - x2^2);  
        %a = d >= 1 && y < 0
    end
    x = 2*x1*sqrt(1 - x1^2 - x2^2);
    z = 1-2*(x1^2 + x2^2);
    point = [point; [x*r y*r z*r]];
end

hold on
plot3(point(:,1), point(:,2), point(:,3), 'xb')