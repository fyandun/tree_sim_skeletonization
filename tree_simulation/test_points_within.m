function [count_par_prev, count_par_act, points_prev, points_act] = test_points_within(direction_par_prev, direction_par_act, radius, len, points, orig)
%
count_par_prev = 0;
count_par_act = 0;
points_prev = [];
points_act = [];
for j=1:size(points, 1)

    test_pt = points(j,:);
    if any(isinf(test_pt))
        continue
    end
    dpt1_test = test_pt - orig;
    proy_par_prev = dot(dpt1_test, direction_par_prev);
    proy_par_act = dot(dpt1_test, direction_par_act);
    
    if abs(proy_par_prev) < len/2 %originally len/2
        % dentro de la longitud del cylindro
        dist_sq = sum(dpt1_test.^2) - proy_par_prev^2;
        if dist_sq <radius^2
            %el punto está dentro del cilindro
            count_par_prev = count_par_prev + 1;
            points_prev = [points_prev; test_pt];
        end
    end
    
    if abs(proy_par_act) < len/2 %originally len/2
        dist_sq = sum(dpt1_test.^2) - proy_par_act^2;
        if dist_sq <radius^2
            %el punto está dentro del cilindro
            count_par_act = count_par_act + 1;
            points_act = [points_act; test_pt];
        end        
        
    end
end

%debugging
% plot3(test_pt(1), test_pt(2), test_pt(3), 'db')