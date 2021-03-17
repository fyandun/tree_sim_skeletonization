classdef tree_old < handle
    
    properties (GetAccess = 'public', SetAccess = 'private')
        n_leav
        root
        trunk_lenght
        branches
        
        pivot    % point until the trunk groes vertically after searching in other directions
        leaves   % strutcture to save all the leaves in the tree
        leaves_duplicate
        max_dist  %100 arbol grandote
        min_dist  %25 arbol grandote
        
        fig_handle
        
        ids 
    end
   
    methods
        function obj = tree_old(points)
%             if ~isempty(filename)               
%                 load(strcat('/home/fyandun/Documentos/Manipulator/fractar_tree_gen/space_colonization_3D _physics _real_data/', filename, '.mat'))
%                 obj.n_leav = length(leaves_pos);
%                 obj.pivot = pivot;
%                 if isempty(pivot)
%                     obj.max_dist = 1;
%                 else
%                     obj.max_dist = 0.01;
%                 end
%                 %vine values
% %                 obj.max_dist = 0.1;
% %                 obj.min_dist = 0.005;
%                 
% %                 %trees from dataset
% % %                 obj.max_dist = 0.1;
%                 obj.min_dist = 0.0005;             
%             else
%                 obj.n_leav = 50;
%                 obj.pivot = [];
%                 obj.max_dist = 100;
%                 obj.min_dist = 25;
%             end
            obj.n_leav = length(points)-1;
            obj.max_dist = 0.1;
            obj.min_dist = 0.05;
            
            obj.fig_handle = figure;

            
            obj.leaves = cell(1,obj.n_leav-1);
            obj.branches = []; %cell(1, 100000); % to have enough space for the branches
            obj.ids = 1;
            
            [~, I] = min(points(:,3));
            root_pos = points(I,:);
            points(I,:) = [];
            
            for i=1:obj.n_leav
                if isempty(points)
                    obj.leaves{i} = leaf(); %to create random leaves 
                else                    
                    pos = points(i,:);
                    obj.leaves{i} = leaf_load(pos,i); % to load leaves positions from file
                end
            end
            obj.leaves_duplicate = obj.leaves;
           
           figure(obj.fig_handle)
           hold on
           for i=1:length(obj.leaves)
              plot3(obj.leaves{i}.position(1), obj.leaves{i}.position(2), obj.leaves{i}.position(3), 'xr') 
           end            
            
%             obj.root = branch([], [100 100 0], [0 0 1], obj.ids, 0); %arbol_grandote   

            obj.root = branch([], root_pos, [0 0 1], obj.ids, 0); %root_pos.Position, pivot
            
            
            
            %obj.branches{1} = obj.root;
            obj.branches = [obj.branches; obj.root];
            
            current = obj.root;
            found = false;
            
            while ~found
                for i=1:obj.n_leav
                   d = pdist2(current.branch_position, obj.leaves{i}.position);
                   if d < obj.max_dist, found = true; break; end
                end           
            
                if ~found    
%                     if ~isempty(obj.pivot)
%                        d_pivot = pdist2(current.branch_position, obj.pivot);
%                        if d_pivot < 0.2 %0.2vine
% %                            obj.max_dist = 1; %vine
%                            obj.max_dist = 1; %tree_pc
%                        end
%                     end
                    epsilon_dir = -0.03 + (0.03 + 0.03)*rand(1, 3);
                    current.direction = current.direction + epsilon_dir;
                    obj.ids = obj.ids + 1;
                    current = current.next(obj.ids, obj.ids-1); %verificar acá!                    
                    obj.branches = [obj.branches; current];
                end
            end
            obj.trunk_lenght = length(obj.branches);
            
        end
        
        function grow(obj)  
                        
            init_grow = 1;
            count = 1;
            while count<=length(obj.leaves_duplicate)
                closestBranch = [];
                record = 100000;
                this_leaf =  obj.leaves_duplicate{count};
                plot3(this_leaf.position(1), this_leaf.position(2), this_leaf.position(3), '*g') %debugging
                
                for j=init_grow:length(obj.branches)
                    this_branch = obj.branches(j);
                    d = pdist2(this_leaf.position, this_branch.branch_position, 'squaredeuclidean');
                    if d < obj.min_dist
                        closestBranch = [];
                        this_leaf.reached = true;
                        obj.leaves(count) = [];
                        break
                    elseif (d < record)
                        closestBranch = this_branch;
                        closestBranch_index = j;
                        record = d;
                    end
                end
                
                if ~isempty(closestBranch)
                   newDir_ = this_leaf.position - closestBranch.branch_position;
                   newDir = newDir_/norm(newDir_);
                   obj.branches(closestBranch_index).direction = newDir;
%                    obj.branches(closestBranch_index).direction = obj.branches(closestBranch_index).direction + newDir;
                   obj.branches(closestBranch_index).count = obj.branches(closestBranch_index).count + 1; 

                end
                count = count + 1;
            end
            obj.leaves_duplicate = obj.leaves;
            
%             for i=length(obj.leaves):-1:1
%                 if obj.leaves{i}.reached
%                     obj.leaves(i) = [];
%                 end                
%             end
            
            for j=length(obj.branches):-1:init_grow
               this_branch = obj.branches(j);               
               if (this_branch.count > 0)
                   obj.ids = obj.ids + 1;
                   epsilon = rand(1,3)*0.03;
                   this_branch.direction = this_branch.direction/(this_branch.count+1) + epsilon;                                     
%                    newPosition = this_branch.position + this_branch.direction;
%                    newBranch = branch(this_branch, newPosition, this_branch.direction);
                   obj.branches = [obj.branches; this_branch.next(obj.ids, j, [], [])];     
                   this_branch.update_ancestors();
                   this_branch.reset;
               end               
            end
            
            
        end
        
        
        function dynSim(obj, t)
%             obj.expand_dynamics(wrench)
            obj.regrow(t);
        end
        
        function regrow(obj,t)
           for j = 2:length(obj.branches)
               this_joint = obj.branches(j);
               this_joint.sping_evol(t);               
           end
        end
        
        
        function expand_dynamics(obj, wrench_)
            
            for j=length(obj.branches):-1:2
                this_joint = obj.branches(j);
                if ~isempty(wrench_)
                    if j == wrench_.branch_id
                        wrench = wrench_;
                    else
                        wrench = [];
                    end
                else
                     wrench = [];
                end
                
                if this_joint.num_child == 0
                    this_joint.dynUpdate_noChild(wrench);                    
                else        
                    this_joint.dynUpdate(obj.branches(this_joint.children), wrench);
               end               
            end            
            
        end
        
        function change_init_conditions(obj)
            for j=2:length(obj.branches)
               obj.branches(j).init_angles = obj.branches(j).theta_all(:,end);
               obj.branches(j).init_angles_prime = obj.branches(j).theta_prime_all(:,end);
               obj.branches(j).theta_all = [];
               obj.branches(j).theta_prime_all = [];
            end
            
        end   
        
        function draw_leaves(obj)
           for i=1:length(obj.leaves_duplicate)
              plot3(obj.leaves_duplicate{i}.position(1), obj.leaves_duplicate{i}.position(2), obj.leaves_duplicate{i}.position(3), 'xr', 'MarkerSize', 10) 
           end            
        end
        
        function draw(obj)
           figure(obj.fig_handle);
           hold on
                      
%            for i=1:length(obj.leaves)
%               plot3(obj.leaves{i}.position(1), obj.leaves{i}.position(2), obj.leaves{i}.position(3), 'xr') 
%            end

%            clf(obj.fig_handle,'reset')
           view(-75,25)
           for i=1:length(obj.branches)  
              thin = 1;
              gross = 6;              
%               m = (thin-gross)/length(obj.branches);
%               b = gross - m;
%               sw = m*i+b;
              m = (gross-thin)/length(obj.branches); %the maximum number of desc is all the branches
              b = thin;            
              sw = m*obj.branches(i).num_descendents + b;
                             
              obj.branches(i).show(sw, 1);              
           end                                  
        end                   
        
        function draw_label(obj)
           fig = figure;
           hold on
           dcm_obj = datacursormode(fig);
           
%            hold on            
%            for i=1:length(obj.leaves)
%               plot3(obj.leaves{i}.position(1), obj.leaves{i}.position(2), obj.leaves{i}.position(3), 'xr') 
%            end

           view(-75,25)
           all_positions = [];
           for i=1:length(obj.branches)
              thin = 1;
              gross = 6;              
%               m = (thin-gross)/length(obj.branches);
%               b = gross - m;
%               sw = m*i+b;
              m = (gross-thin)/length(obj.branches); %the maximum number of desc is all the branches
              b = thin;            
              sw = m*obj.branches(i).num_descendents + b;
                             
              obj.branches(i).show(sw, 0);    
              h = plot3(obj.branches(i).branch_position(1), obj.branches(i).branch_position(2), obj.branches(i).branch_position(3), '.k');
              h.UserData = i;
%               all_positions = [all_positions; obj.branches(i).branch_position]; 
           end      
%            plot3(all_positions(:,1), all_positions(:,2), all_positions(:,3), '.b');
           set(dcm_obj,'UpdateFcn',@myupdatefcn)
           
        end
        
        function draw_cylinders(obj)
           figure
           hold on
%            for i=1:length(obj.leaves)
%               plot3(obj.leaves{i}.position(1), obj.leaves{i}.position(2), obj.leaves{i}.position(3), 'xg') 
%            end
           
           for i=2:length(obj.branches)  
              thin = 0.02;
              gross = 0.06;  %estaba 2 para el arbol gigantesco            
%               m = (thin-gross)/length(obj.branches);
%               b = gross - m;
%               sw = m*i+b;
              m = (gross-thin)/length(obj.branches);
              b = thin;            
              new_rad = m*obj.branches(i).num_descendents + b;
              obj.branches(i).update_Inertia(new_rad);
              
              obj.branches(i).show_cylinders(new_rad); 
           end
           
        end 
        

        
    end
end
    
