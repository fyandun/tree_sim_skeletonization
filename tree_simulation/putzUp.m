function orient_mat = putzUp(directional_vector)
%the idea is to keep the detected orientation of the unit vector in the x axis, but put the y
%axis down as in the paper, and infer the z axis.
% do it only for the contact point between the arm and the branch

% rot_mat = zeros(3,3);

unit_z = directional_vector';
unit_x = [1 0 0]'; %y down in global coord
unit_y = cross(unit_z, unit_x);
pre_orient_mat_ = [unit_z unit_y unit_x]; 
orient_mat_ = gs_ortonorm(pre_orient_mat_);
orient_mat = [orient_mat_(:,3) orient_mat_(:,2) orient_mat_(:,1)];