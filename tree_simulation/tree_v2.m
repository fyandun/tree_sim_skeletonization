classdef tree_v2 < matlab.mixin.Copyable
    
    properties (GetAccess = 'public', SetAccess = 'public') %properties (GetAccess = 'public', SetAccess = 'private')
        n_leav
        root
        trunk_lenght
        branches
        branches_duplicate
        branches_pruned
        
        pivot    % point until the trunk groes vertically after searching in other directions
        leaves   % strutcture to save all the leaves in the tree individually
        leaves_duplicate % strutcture to save all the leaves in the tree individually
        leaves_all
        leaves_all_duplicate
        
        max_dist  %100 arbol grandote
        min_dist  %25 arbol grandote
        
        height
        
        indices_ids
        
        fig_handle
        fig_handle_cyls
        
        ids 
    end
   
    methods
        function obj = tree_v2(points)

            obj.n_leav = length(points)-1;
            obj.max_dist = 0.5; %0.8 for tree1_2_3_skeleton, 
            obj.min_dist = 0.01; 
            
            obj.fig_handle = figure;
            obj.fig_handle_cyls = figure;
            obj.indices_ids = 1;
            
            obj.leaves = cell(1,obj.n_leav-1);            
            obj.branches = []; %cell(1, 100000); % to have enough space for the branches
            obj.ids = 1;
            
            [~, I] = min(points(:,3));
            root_pos = points(I,:);
            points(I,:) = [];
            
            obj.leaves_all = points;
            obj.leaves_all_duplicate = obj.leaves_all;
            obj.height = max(points(:,3)) - min(points(:,3));
            
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
                    epsilon_dir = -0.03 + (0.03 + 0.03)*rand(1, 3);
                    current.direction = current.direction + epsilon_dir;
                    obj.ids = obj.ids + 1;
                    current = current.next(obj.ids, obj.ids-1, [], []); %verificar acá!                    
                    obj.branches = [obj.branches; current];
                end
            end
            obj.trunk_lenght = length(obj.branches);
            obj.branches_duplicate = obj.branches;
            
        end
        
        function grow(obj)  
         cuenta = 1;         
         
         while ~isempty(obj.branches_duplicate)
            radius_search = 0.1; %0.5 tree1_skeleton, 1 tree2_3_skeleton_modified, 1 with citiblok dist, 0.5 tree6_skeleton, 0.05 for lab vine,  0.5 for the buds test
                     
            current_node = obj.branches_duplicate(1);  
            
            %debugging
%             plot3(current_node.branch_position(1), current_node.branch_position(2), current_node.branch_position(3), '*g' )
%             obj.draw
%          if cuenta == 199 %92, 186, 187, 188, 199
%              asda=1;
%          end
            
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
                    if radius_search > 1 %0.1 for tree1_2_3_skeleton, 3 tree6_skeleeton, 0.2 for the lab vine, 3 for the bud test
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

                   %check if that node has other parent
                   aux = Idx{1}(j)+1 == obj.indices_ids;                                             
                   if ~any(aux) 
                       obj.indices_ids = [obj.indices_ids Idx{1}(j)+1]; %aqui debo chequear si está repetido, si no está repetido no mas le anado
                       obj.branches = [obj.branches; current_node.next(Idx{1}(j)+1, current_node.id, pos, dir)];
    %                    current_node.update_ancestors_plus;
                       current_node.update_ancestors();
                   else
                      
                      multi_parent = obj.branches(aux);
                      dir_par_prev_ = multi_parent.branch_position - multi_parent.branch_parent.branch_position;
                      norm_dir_par_prev = norm(dir_par_prev_);
                      dir_par_prev = dir_par_prev_/norm_dir_par_prev; %unit vector
                      
                      dir_par_act_ = multi_parent.branch_position - current_node.branch_position;
                      norm_dir_par_act = norm(dir_par_act_);
                      dir_par_act = dir_par_act_/norm_dir_par_act; %unit vector
                      
                      rad_cyl_test = 0.02; %0.2 para tree6_skeleton
                      len_cyl_test = obj.height/2; %25 para tree6_skeleton

%                       debugging
%                       plot3(multi_parent.branch_position(:,1), multi_parent.branch_position(:,2), multi_parent.branch_position(:,3), 'xk')
%                       plot3(multi_parent.branch_parent.branch_position(:,1), multi_parent.branch_parent.branch_position(:,2), multi_parent.branch_parent.branch_position(:,3), '<m')
%                       plot3(points_prev(:,1), points_prev(:,2), points_prev(:,3), 'sm')
%                       plot3(points_act(:,1), points_act(:,2), points_act(:,3), 'dm')
%                       quiver3(multi_parent.branch_position(:,1), multi_parent.branch_position(:,2), multi_parent.branch_position(:,3), ...
%                           dir_par_prev(1), dir_par_prev(2), dir_par_prev(3),'g'), dir_par_act(1), dir_par_act(2), dir_par_act(3),'g')
                        
                      thres_neighs = 5; %5 para tree6_skeleton
                      
                      if norm_dir_par_prev <= norm_dir_par_act
                        ratio_prev_act = norm_dir_par_prev/norm_dir_par_act;
                        if ratio_prev_act < 0.7 %0.7
                          continue              
                        else
                            %check the gradient cyl
                            % was originally obj.leaves_all
                            [count_par_prev, count_par_act, points_prev, points_act] = test_points_within(dir_par_prev, dir_par_act, rad_cyl_test, len_cyl_test, obj.leaves_all_duplicate, multi_parent.branch_position);
                            if count_par_prev - count_par_act < -thres_neighs
                                %update parent and son
                                dir = pos - current_node.branch_position;                                                                  
                                parent_node = multi_parent.branch_parent;

                                if ~isempty(parent_node)
                                  parent_node.delete_child(Idx{1}(j)+1);
                                  parent_node.update_ancestors_min();
                                end
                                  multi_parent.update_properties(dir,  current_node);   
                                  current_node.update_ancestors();                        
                                  %debo actualizar el numero de hijos y el indice de
                                  %los hijos
                                  current_node.update_child_info(Idx{1}(j)+1)                                                                                                
                            else
                                if count_par_prev - count_par_act <= 0
                                    disp('weak decision, based on available info')
                                end
                                if count_par_prev - count_par_act > 0 && count_par_prev - count_par_act < thres_neighs
                                    disp('weak decision, based on available info')
                                end                                
                                continue                                                                                                
                            end                            
                        end
                      else
                         ratio_prev_act = norm_dir_par_act/norm_dir_par_prev;
                         if ratio_prev_act < 0.7 %0.7
                              %update parent and son
                              dir = pos - current_node.branch_position;                                                                  
                              parent_node = multi_parent.branch_parent;

                              if ~isempty(parent_node)
                                parent_node.delete_child(Idx{1}(j)+1);
                                parent_node.update_ancestors_min();
                              end
                                multi_parent.update_properties(dir,  current_node);   
                                current_node.update_ancestors();                        
                                %debo actualizar el numero de hijos y el indice de
                                %los hijos
                                current_node.update_child_info(Idx{1}(j)+1)                             
                         else
                            %test cyl
                            % was originally obj.leaves_all
                            [count_par_prev, count_par_act, points_prev, points_act] = test_points_within(dir_par_prev, dir_par_act, rad_cyl_test, len_cyl_test, obj.leaves_all_duplicate, multi_parent.branch_position);
                            if count_par_prev - count_par_act > thres_neighs
                                continue
                            else
                                if count_par_prev - count_par_act > 0
                                    disp('weak decision, based on available info')
                                end                                
                                if count_par_prev - count_par_act <= 0 && count_par_prev - count_par_act > -thres_neighs
                                    disp('weak decision, based on available info')
                                end                                                                    
                                %update parent and son
                                dir = pos - current_node.branch_position;                                                                  
                                parent_node = multi_parent.branch_parent;

                                if ~isempty(parent_node)
                                  parent_node.delete_child(Idx{1}(j)+1);
                                  parent_node.update_ancestors_min();
                                end
                                  multi_parent.update_properties(dir,  current_node);   
                                  current_node.update_ancestors();                        
                                  %debo actualizar el numero de hijos y el indice de
                                  %los hijos
                                  current_node.update_child_info(Idx{1}(j)+1)                                                                                                
                            end                             
                         end                                                
                      end                                                                                                                                                                                                                                 
                   end    
%                    %debugging
%                     obj.draw
%                     ax = gca;               % get the current axis
%                     ax.Clipping = 'off';    % turn clipping off
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
               %obj.branches(j).K = obj.branches(j).first_guess_k./[1 0.015 0.015];
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
                  
           %Debugging
           clf(obj.fig_handle,'reset')
           plot3(obj.leaves_all_duplicate(:,1), obj.leaves_all_duplicate(:,2), obj.leaves_all_duplicate(:,3), 'xr')
           hold on
%            for i=1:length(obj.leaves)
%               plot3(obj.leaves{i}.position(1), obj.leaves{i}.position(2), obj.leaves{i}.position(3), 'xr') 
%            end       
           
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
           
           figure(obj.fig_handle_cyls)
           
%            for i=1:length(obj.leaves)
%               plot3(obj.leaves{i}.position(1), obj.leaves{i}.position(2), obj.leaves{i}.position(3), 'xg') 
%            end
           
           for i=2:length(obj.branches)  
              thin = 0.015; %was 0.003
              gross = 0.03;  %0.06 tree1,   0.3 tree2 and tree3, 0.5 tree6        
%               m = (thin-gross)/length(obj.branches);
%               b = gross - m;
%               sw = m*i+b;
              m = (gross-thin)/length(obj.branches);
              b = thin;            
              new_rad = m*obj.branches(i).num_descendents + b;
              obj.branches(i).update_Inertia(new_rad);
              
              obj.branches(i).show_cylinders(new_rad); 
              hold on
           end
           hold off
        end  
        
        function copy_branches(obj)
           obj.branches_pruned = obj.branches;                         
        end
        
        function remove_this_link(obj, index_to_remove)
            obj.branches_pruned(index_to_remove) = [];
            obj.indices_ids(index_to_remove) = [];
        end
        
        function set_pruned_indicex_ids(obj, new_indices)
           obj.indices_ids = new_indices;
        end
        
        function update_branches_to_pruned(obj)
            obj.branches = obj.branches_pruned;
        end
        
    end
end
    
