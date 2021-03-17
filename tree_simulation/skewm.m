function mat = skewm(vect)

x = vect(1);
y = vect(2);

if length(vect) >2 
    z = vect(3);
else
    z = 0;
end
mat = [0 -z y; ...
    z 0 -x; ...
    -y x 0];
