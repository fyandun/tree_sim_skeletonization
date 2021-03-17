function kd = first_guess_k(perimeter, n_links)

total_length_brach = 0.5; %hand measured [m]

l = total_length_brach/n_links;

K_ = 1000;
nu = 0.3; %poisson_ratio see https://www.fpl.fs.fed.us/documnts/fplgtr/fplgtr113/ch04.pdf 
E = 10e9; %young_mod wood (eye average of values seen 
% in https://www.engineeringtoolbox.com/timber-mechanical-properties-d_1789.html )

radius = perimeter/(2*pi);
G = E/(2*(1+nu));

% see https://en.wikipedia.org/wiki/List_of_second_moments_of_area
In = (1/4)*pi*radius^4; %y local direction
Ib = (1/4)*pi*radius^4; %z local direction
It = K_* (1/4)*pi*radius^4; %x local direction , should be greater than the other two 
%(it is harder to twist along the local x direction)

k_t = G * It; 
k_b = E * Ib;
k_n = E * In;

K = [k_t k_b k_n];
kd = K/l;