function steady_positions_final = appply_tfs(points_struct, tf_rot, tf_trasl)
%ids_all = [1 3 4 6 12 13 14 15];
steady_positions_final = struct([]);

for i =1:length(points_struct)
    aux_ = tf_rot*[points_struct(i).position;1];
    steady_positions_final(i).Id = points_struct(i).Id;
    if ~ isnan(tf_trasl{i})
        aux = aux_(1:3)' - tf_trasl{i};
        steady_positions_final(i).position = aux;
    end
    
end

