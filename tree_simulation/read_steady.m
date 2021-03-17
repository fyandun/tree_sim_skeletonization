function marker_pos = read_steady(filename, Tform_init)

bag_init_conds = rosbag(filename);
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
            pos = Tform_init*[singleTag.Pose.Pose.Pose.Position.X singleTag.Pose.Pose.Pose.Position.Y singleTag.Pose.Pose.Pose.Position.Z 1]'; 
            pos(3) = pos(3) - 0.06; %to compensate for the bias between pos of marker 5 and the point cloud bottom 
            orient = [singleTag.Pose.Pose.Pose.Orientation.W singleTag.Pose.Pose.Pose.Orientation.X singleTag.Pose.Pose.Pose.Orientation.Y singleTag.Pose.Pose.Pose.Orientation.Z];
            
            if i ==1
                marker_pos(aux).position = pos(1:3); 
                marker_pos(aux).orientation = orient;
            else
                marker_pos(aux).position = mean([marker_pos(aux).position pos(1:3)],2);
                marker_pos(aux).orientation = mean([marker_pos(aux).orientation; orient]);
            end
        end
    end
    
end
