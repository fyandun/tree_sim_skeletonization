classdef tree < handle
    
    properties (GetAccess = 'public', SetAccess = 'private')
        n_leav
        root
        trunk_lenght
        branches
        branches_duplicate
        
        pivot    % point until the trunk groes vertically after searching in other directions
        leaves   % strutcture to save all the leaves in the tree individually
        leaves_duplicate % strutcture to save all the leaves in the tree individually
        leaves_all
        
        max_dist  %100 arbol grandote
        min_dist  %25 arbol grandote
        
        indices_ids  %used to keep track of the indices of each branch
        
        fig_handle
        
        ids 
    end
   
    methods
        function obj = tree(points)

            obj.n_leav = length(points)-1;
            obj.max_dist = 0.5; %0.1 for the scaled version
            obj.min_dist = 0.01; 
            
            obj.fig_handle = figure;
            obj.indices_ids = 1;
            
            obj.leaves = cell(1,obj.n_leav-1);            
            obj.branches = []; %cell(1, 100000); % to have enough space for the branches
            obj.ids = 1;
            
            [~, I] = min(points(:,3));
            root_pos = points(I,:);
            points(I,:) = [];
            
            obj.leaves_all = points;
            
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
            obj.branches_duplicate = obj.branches;
            
        end
        
        function grow(obj)  
         cuenta = 1;
         while ~isempty(obj.branches_duplicate)
            radius_search = 0.05; %0.1 for the scaled tree, 0.5 for the megacloud16, 0.08 if the cloud has more points   
                     
            current_node = obj.branches_duplicate(1);  
            
            %debugging
%             plot3(current_node.branch_position(1), current_node.branch_position(2), current_node.branch_position(3), '*g' )

            if ~isempty(current_node.branch_parent)                
                aux = current_node.branch_position == obj.leaves_all; %optimizar esta cuestion
                % aux = all(aux,2);
                % obj.leaves_all(aux) = [];
                obj.leaves_all(aux) = inf;
                if ~any(isfinite(obj.leaves_all(:))), break, end
            end
            
%             Mdl = KDTreeSearcher(obj.leaves_all); 
            Mdl = ExhaustiveSearcher(obj.leaves_all);
                        
            found = false;
            while ~found                
                Idx = rangesearch(Mdl, current_node.branch_position, radius_search);
                if ~isempty(Idx{1})
                    found = true;
                else
                    radius_search = radius_search * 1.05;
                    if radius_search > 0.1
                        break
                    end
                end
            end            
            
             if ~isempty(Idx{1})
                for j = 1:length(Idx{1})
    %                tmp = 1:length(obj.branches);               
                   %obj.ids = obj.ids + 1;
                   pos =  obj.leaves_all(Idx{1}(j),:);
                   dir = pos - current_node.branch_position;

                   %check if that node has othe parent
                   aux = Idx{1}(j)+1 == obj.indices_ids;                                             
                   if ~any(aux) 
                       obj.indices_ids = [obj.indices_ids Idx{1}(j)+1]; %aqui debo chequear si está repetido, si no está repetido no mas le anado
                       obj.branches = [obj.branches; current_node.next(Idx{1}(j)+1, current_node.id, pos, dir)];
    %                    current_node.update_ancestors_plus;
                       current_node.update_ancestors();
                   else
                      multi_parent = obj.branches(aux);
                      dist_par_prev = norm(multi_parent.branch_position - multi_parent.branch_parent.branch_position);
                      dist_par_act = norm(multi_parent.branch_position - current_node.branch_position);
                      if dist_par_prev < dist_par_act
                          continue
                      else                      
                          %update parent and son
                          dir = pos - current_node.branch_position;                                                                  
    %                       parent_node = obj.branches(obj.indices_ids==multi_parent.parent_id).branch_parent;
                          parent_node = multi_parent.branch_parent;


                          if ~isempty(parent_node)
                            parent_node.delete_child(Idx{1}(j)+1);
                            parent_node.update_ancestors_min();
                          end
                            multi_parent.update_properties(dir,  current_node);   
                            current_node.update_ancestors();                        
    %                         current_node.update_ancestors_plus;

                            %debo actualizar el numero de hijos y el indice de
                            %los hijos
                            current_node.update_child_info(Idx{1}(j)+1)
    %                         current_node.update_ancestors()                        
                      end                                        
                   end    
                   %debugging
    %                obj.draw
                end
             end
            
            obj.branches_duplicate = obj.branches;                      
            
            obj.branches_duplicate(1:cuenta) = [];
            cuenta = cuenta +1;
         end                      
            
        end
        
        
        function dynSim(obj, t)
%             obj.expand_dynamics(wrench)
            obj.regrow(t);
        end
        
        function regrow(obj,t)
           for j = 2:length(obj.branches)
               this_joint = obj.branches(j);
               try
                this_joint.sping_evol(t);
               catch
                   a = 1;
               end
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
                    this_joint.dynUpdate(obj.branches(sum(obj.indices_ids == this_joint.children',1, 'native')), wrench);
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
              h.UserData = [i obj.indices_ids(i)];
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
    
