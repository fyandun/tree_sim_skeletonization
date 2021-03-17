function orient_mat = putyDown(directional_vector)
%the idea is to keep the detected orientation of the unit vector in the x axis, but put the y
%axis down as in the paper, and infer the z axis.
% do it only for the contact point between the arm and the branch

% rot_mat = zeros(3,3);

unit_x = directional_vector';
unit_y = [0 0 -1]'; %y down in global coord
unit_z = cross(unit_x, unit_y);
pre_orient_mat_ = [unit_x unit_y unit_z]; 
orient_mat = gs_ortonorm(pre_orient_mat_);
   
