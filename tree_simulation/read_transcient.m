function marker_pos = read_transcient(filename, Tform_init)

bag_init_conds = rosbag(filename);
bSel_init = select(bag_init_conds,'Topic','/tag_detections');
msgStructs = readMessages(bSel_init,'DataFormat','struct');
ids_all = [1 3 4 6 12 13 14 15];

marker_pos = struct([]);
init_times_all = zeros(1,length(ids_all));
 
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
            ros_time = double(singleTag.Pose.Header.Stamp.Sec) + double(singleTag.Pose.Header.Stamp.Nsec)*1e-9;
            
            if init_times_all(aux) == 0 
                init_times_all(aux) = ros_time; 
                marker_pos(aux).time = 0;
                marker_pos(aux).position = pos(1:3);
                marker_pos(aux).range = norm(pos(1:3));
                marker_pos(aux).orientation = orient;                
            else
                marker_pos(aux).position = [marker_pos(aux).position pos(1:3)];
                marker_pos(aux).range = [marker_pos(aux).range norm(pos(1:3))];
                marker_pos(aux).orientation = [marker_pos(aux).orientation orient];
                marker_pos(aux).time = [marker_pos(aux).time ros_time-init_times_all(aux)];
            end
        end
    end
    
end
