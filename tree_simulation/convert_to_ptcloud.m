function xyz_all = convert_to_ptcloud(tree, no_samples)
xyz_all = [];

for j = 2:length(tree.branches)
    orientation = tree.branches(j).orientation;
    init_pos = tree.branches(j).branch_parent.branch_position;
    end_pos = tree.branches(j).branch_position;
    radius = tree.branches(j).radius;
    
    sample_rad = radius*rand(1,no_samples);
    sample_ang = 2*pi*rand(1,no_samples);
    sample_length = norm(end_pos - init_pos)*rand(1,no_samples);
    
    x_ = sample_length;
    y_ = sample_rad.*cos(sample_ang);
    z_ = sample_rad.*sin(sample_ang);
    
    xyz_ = [x_;y_;z_];
    
    xyz = orientation*[x_;y_;z_] + tree.branches(j).branch_parent.branch_position';
    
    xyz_all = [xyz_all;xyz'];
    
end


