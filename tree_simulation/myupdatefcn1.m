function [text, pos] = myupdatefcn1(~, event_obj, path_save, label_case)
    pos = get(event_obj,'Position');
    
%     I = get(event_obj.Target, 'UserData');   
%     text = {['X: ',num2str(pos(1))],...
%             ['Y: ',num2str(pos(2))], ...
%             ['Z: ',num2str(pos(3))], ...
%             ['Id: ',num2str(I)]};   
        
    text = {['X: ',num2str(pos(1))],...
            ['Y: ',num2str(pos(2))], ...
            ['Z: ',num2str(pos(3))]};
                 

    m = matfile(path_save, 'Writable', true);
    
    if label_case == 1 %leaves    
        m.leaves_pos = [m.leaves_pos;pos];
    elseif label_case == 2 %root
        root_pos.Position = pos;
        m.root_pos = root_pos;
    elseif label_case == 3 %pivot
        m.pivot = pos;
    end
        
    hold on
    plot3(pos(1), pos(2), pos(3), 'xr')

end