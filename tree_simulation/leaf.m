classdef leaf < handle
    
    properties (GetAccess = 'public', SetAccess = 'private')
       position  
    end
    
    properties (GetAccess = 'public', SetAccess = 'public')
       reached =  false;  
    end    
    
    methods
       
        function obj = leaf()
%            obj.position = [randi(200) randi(200) randi([80 200])];
            obj.position = [randi(20) randi(20) randi([8 20])];
        end
        
    end
    
    
end
