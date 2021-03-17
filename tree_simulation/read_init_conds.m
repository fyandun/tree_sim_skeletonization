function [marker_pos, Tbp_c] = read_init_conds(bag_path)

init_file = 'initial_conditions';
filename_init = strcat(bag_path, init_file, '.bag');
bag_init_conds = rosbag(filename_init);
bSel_init = select(bag_init_conds,'Topic','/tag_detections');
msgStructs = readMessages(bSel_init,'DataFormat','struct');
ids_all = [1 3 4 5 6 12 13 14 15];

marker_pos = struct([]);
for i = 1:length(msgStructs)
    singleMessage = msgStructs{i};
    for j=1:length(singleMessage.Detections)
        singleTag = singleMessage.Detections(j);
        aux = find(ids_all == singleTag.Id);  
        if ~isempty(aux)
            marker_pos(aux).Id = singleTag.Id;
            pos = [singleTag.Pose.Pose.Pose.Position.X singleTag.Pose.Pose.Pose.Position.Y singleTag.Pose.Pose.Pose.Position.Z];
            orient = [singleTag.Pose.Pose.Pose.Orientation.W singleTag.Pose.Pose.Pose.Orientation.X singleTag.Pose.Pose.Pose.Orientation.Y singleTag.Pose.Pose.Pose.Orientation.Z];
            
            if i ==1
                marker_pos(aux).position = pos; 
                marker_pos(aux).orientation = orient;
            else
                marker_pos(aux).position = mean([marker_pos(aux).position; pos]);
                marker_pos(aux).orientation = mean([marker_pos(aux).orientation; orient]);
            end
        end
    end
    
end

%%% now put everything in trunk coordinates 
base_index = 4;
Tc_base = eye(4);
Tc_base(1:3,4) = marker_pos(base_index).position';
quat_c_base = quaternion(marker_pos(base_index).orientation);
Tc_base(1:3,1:3) = quat2rotm(quat_c_base);
Tbase_c = inv(Tc_base);

Tb_bp = eye(4);
Tb_bp(1:3,1:3) = rotz(-90)*roty(-90);
% Tbp_b = inv(Tb_bp);

Tbp_c = Tb_bp \ Tbase_c;
for i=1:length(ids_all)
   if i == base_index
       marker_pos(i).position = [0 0 0 1]';
       continue
   end
   marker_pos(i).position = Tbp_c * [marker_pos(i).position';1]; 
   marker_pos(i).position(3) = marker_pos(i).position(3) - 0.06; %to compensate for the bias between pos of marker 5 and the point cloud bottom 
    
end