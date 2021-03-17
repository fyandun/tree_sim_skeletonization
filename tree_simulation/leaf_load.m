classdef leaf_load < handle
    
    properties (GetAccess = 'public', SetAccess = 'private')
       position  
       index
    end
    
    properties (GetAccess = 'public', SetAccess = 'public')
       reached =  false;  
    end    
    
    methods
       
        function obj = leaf_load(position, index)
%             load('/home/fyandun/Documentos/Manipulator/fractar_tree_gen/space_colonization_3D _physics/leaves_vine_1.mat', 'leaves_pos')
            obj.position = [position(1) position(2) position(3)];
            obj.index = index;
        end
        
    end
    
    
end
