classdef branch < handle
    properties (GetAccess = 'public', SetAccess = 'public')
       %%% Properties used for the tree grow simulation
       id                 %used to keep track o the childs of this branch
       parent_id          %used to keep track o the parent of this branch       
       branch_position    %ending position of the branch segment [x y]
       branch_position_orig
       branch_parent      % begining positino of the branch segment [x y]
       orig_direction
       direction
       num_child;         %amount of children for this branch(joint)
       num_descendents   %%amount of descendents for this branch(joint)
       count = 0;
       len  
       effective_len
       radius
       children
       
       K                  %spring stifness constants 
       beta               %parameter related to the velocity in the second order equation
       alpha
       init_angles        % initial condition for the diferential equation
       init_angles_prime  % initial condition for the diferential equation
              
       %%% Properties used for the dynamic simulation
       com 
       mass 
       density           %depend on the type of wood, better than setting the mass -> https://www.engineeringtoolbox.com/wood-density-d_40.html
       gravity
       
       velocity
       accel
       Inertia
       orientation
       orientation_orig
       orientation_modified
       
       orientation_rel
       orientation_rel_orig
       ang_vel
       ang_accel
       
       mass_c_hat
       mass_p_hat              
       ppm
       op_hat
       I_p_w_hat
       
       force
       torque
       
       force_hat
       torque_hat
       
       theta_all
       theta_prime_all

    end
    
    
    methods
        function obj = branch(parent, pos, dir, id, parent_id)
            obj.density = 600; %600            
            obj.mass = obj.density*obj.len*2*pi*obj.radius^2;
%             obj.effective_len = obj.len;
            
            %vine values
            obj.radius = 0.04;  %1 arbol grandote
            obj.len = 0.1;  %2 arbol grandote
%             %tree values
%             obj.radius = 0.2; %1 arbol grandote
%             obj.len = 0.5; %2 arbol grandote
            
            obj.branch_position = pos;
            obj.branch_position_orig = pos;
            obj.branch_parent = parent;
            obj.direction = dir;                        
            obj.orig_direction = dir;
            obj.id = id;
            obj.parent_id = parent_id;            
            obj.num_child = 0;
            obj.num_descendents = 0;
            obj.children = [];
            
            obj.theta_all = [];
            obj.theta_prime_all = [];
            
            obj.init_angles = [0 0 0]'; % ojo que aqui debe ir relacionado con el estado de esta branch
            obj.init_angles_prime = [0 0 0]'; % ojo que aqui debe ir relacionado con el estado de esta branch
            
            obj.alpha = 0; % I think this term is for the stability of the equation solver
            obj.beta = 0.3; %parameter to account for the velocity dependent term in the diff equation 0.1 default
            
            
            obj.gravity = [0, 0, -9.8]'; %por ahora la gravedad va en y
            obj.velocity = [0 0 0]';
            obj.accel = [0 0 0]';
            obj.ang_vel = [0 0 0]';
            obj.ang_accel = [0 0 0]';
            
            if ~isempty(parent)
                 obj.orientation = putyDown(obj.direction);
                 obj.orientation_rel = obj.branch_parent.orientation\obj.orientation;
            else
%                   obj.orientation = [[0 0 1]' [0 1 0]' [-1 0 0]']; %=roty(-90)
                  obj.orientation = eye(3);
                  obj.orientation_rel = obj.orientation;
            end
            obj.orientation_orig = obj.orientation;
            obj.orientation_modified = obj.orientation;
            obj.orientation_rel_orig = obj.orientation_rel;
            
%             obj.K = [3787.70060856787, 2400.7079162819047, 112.400698300269]*10;
            obj.K = obj.first_guess_k;            
            
            if ~isempty(obj.branch_parent)
               obj.com = (pos + parent.branch_position)/2;  
               
                %calculate the inertia for that branch     
                obj.effective_len = norm(obj.branch_position - obj.branch_parent.branch_position);
                obj.update_Inertia(obj.radius);
%                 obj.force = obj.mass*obj.gravity; %force produced by the mass of the branch % OJO AQUI LA GRAVEDAD DEBE IR COMO VECTOR
%                 obj.torque = skewm(obj.com - parent.branch_position)*obj.force; %torque produced only by the mass of the branch
                obj.force = [0 0 0]'; %force produced by the mass of the branch % OJO AQUI LA GRAVEDAD DEBE IR COMO VECTOR
                obj.torque = [0 0 0]'; %torque produced only by the mass of the branch                
            end
                                       
        end
        
        %this function creates the orientation matrix from the orientation
        function make_orient_mat(obj)           
            if ~isempty(obj.branch_parent)
                 obj.orientation = putyDown(obj.direction);
                 obj.orientation_rel = obj.branch_parent.orientation\obj.orientation;
            else
%                   obj.orientation = [[0 0 1]' [0 1 0]' [-1 0 0]']; %=roty(-90)
                  obj.orientation = eye(3);
                  obj.orientation_rel = obj.orientation;
            end
            obj.orientation_orig = obj.orientation;
            obj.orientation_modified = obj.orientation;
            obj.orientation_rel_orig = obj.orientation_rel;            
        end
        
        function update_Inertia(obj, radius)
            volume = 2*pi*radius^2*obj.effective_len;
            obj.mass = volume * obj.density;
            obj.radius = radius;
            Ix = 0.5 * obj.mass * radius^2;
            Iy = (1/12)*obj.mass * (3 * radius^2 + obj.effective_len^2);
            Iz = Iy;
            Inertia_ = diag([Ix, Iy, Iz]);
            obj.Inertia = obj.orientation * Inertia_ * obj.orientation'; %inertia in the global frame            
        end
        
        function next_branch = next(obj, id_child, id_parent, position, direction)
           if isempty(direction) || isempty(position)            
               next_dir = obj.direction*obj.len;
               new_pos = obj.branch_position + next_dir;
               new_dir = obj.direction;
           else
               new_pos = position;
               new_dir = direction;
           end
           obj.num_child = obj.num_child + 1;
           obj.children = [obj.children id_child];
           next_branch = branch(obj, new_pos, new_dir, id_child, id_parent);            
        end
        
        function delete_child(obj, child_to_delete)
            aux = obj.children == child_to_delete;
            obj.children(aux) = [];
            obj.num_child = obj.num_child - 1; 
        end
        
        function update_properties(obj, new_dir, new_parent)
%            obj.branch_position  = new_pos;
           obj.direction = new_dir;
           obj.branch_parent = new_parent;
           obj.parent_id = new_parent.id;
           
           obj.orientation = putyDown(obj.direction);
           
           obj.orientation_orig = obj.orientation;
           obj.orientation_modified = obj.orientation;
           obj.orientation_rel = obj.branch_parent.orientation\obj.orientation;
           obj.orientation_rel_orig = obj.orientation_rel;
           
           obj.com = (obj.branch_position + obj.branch_parent.branch_position)/2;           
           
           obj.effective_len = norm(obj.branch_position - obj.branch_parent.branch_position);
           obj.update_Inertia(obj.radius);
           
        end
        
        function update_child_info(obj, child_idx)
           obj.children = [obj.children child_idx];
           obj.num_child = obj.num_child + 1;            
        end
        
        function update_ancestors(obj)
            
            obj.num_descendents = obj.num_descendents + 1;
            
            if obj.parent_id ~=0
                obj.branch_parent.update_ancestors()
            else
                return;
            end
            
        end
        
        function update_ancestors_plus(obj)
            
            obj.num_descendents = obj.num_descendents + 1;
            
%             if obj.parent_id ~=0
%                 obj.branch_parent.update_ancestors_plus()
%             else
%                 return;
%             end
            
        end        
        
        function update_ancestors_min(obj)
            
            obj.num_descendents = obj.num_descendents - 1;
            
            if obj.parent_id ~=0
                obj.branch_parent.update_ancestors_min()
            else
                return;
            end            
        end        

        function dynUpdate_noChild(obj, wrench)
            if isempty(wrench)
               force_ext = zeros(3,1);
               torque_ext = zeros(3,1);
            else
               force_ext = wrench.f_ext;
               torque_ext = wrench.t_ext + cross((obj.branch_position - obj.branch_parent.branch_position)', force_ext);
%                torque_ext = wrench.t_ext + cross(obj.branch_position', force_ext);               
            end
           obj.mass_p_hat = obj.mass;
           obj.I_p_w_hat = obj.Inertia;
           obj.force_hat = obj.force + force_ext;
           obj.torque_hat = obj.torque + torque_ext;
           obj.com = (obj.branch_position + obj.branch_parent.branch_position)/2;
           obj.op_hat = obj.com;
            
        end
        
        function dynUpdate(obj, children_branches, wrench)
            comp_com = [0 0 0];      
            
            if isempty(wrench)
               force_ext = zeros(3,1);
               torque_ext = zeros(3,1);
            else
               force_ext = wrench.f_ext;
               torque_ext = wrench.t_ext + cross((obj.branch_position - obj.branch_parent.branch_position)', force_ext);
%                torque_ext = wrench.t_ext + cross(obj.branch_position', force_ext);
               
            end            
            
            %TODO: improve this, eliminate this for loop
            for j=1:length(children_branches)
                comp_com = comp_com + children_branches(j).op_hat;
            end
            obj.op_hat = (comp_com + obj.com)/(obj.num_child + 1);
            obj.ppm = obj.com - obj.op_hat;
            
            mc_hat = 0;
            I_c_w_hat = zeros(3,3);
            force_c_hat = [0 0 0]';
            torque_c_hat = [0 0 0]';
            for j=1:length(children_branches)
                               
                mc_hat = mc_hat + children_branches(j).mass_p_hat;
                
                pcm_hat = children_branches(j).op_hat - obj.op_hat;
                I_c_w_hat_ = children_branches(j).I_p_w_hat + children_branches(j).mass_p_hat*skewm(pcm_hat)'*skewm(pcm_hat); 
                I_c_w_hat = I_c_w_hat + I_c_w_hat_;
                
                force_c_hat = force_c_hat + children_branches(j).force_hat;
                
%                 torque_c_hat = torque_c_hat + (-skewm(children_branches(j).branch_position - obj.branch_position)*children_branches(j).force_hat + children_branches(j).torque_hat);  
                torque_c_hat = torque_c_hat + (-skewm(obj.branch_parent.branch_position - obj.branch_position)*children_branches(j).force_hat + children_branches(j).torque_hat);   %este me late mas   
%                 torque_c_hat = torque_c_hat + (skewm(obj.branch_parent.branch_position - children_branches(j).op_hat)*children_branches(j).force_hat + children_branches(j).torque_hat);              
                
            end
            
            obj.mass_p_hat = obj.mass + mc_hat;
                        
            obj.I_p_w_hat = obj.Inertia + obj.mass_p_hat*skewm(obj.ppm)'*skewm(obj.ppm) + I_c_w_hat;
            obj.force_hat = obj.force + force_ext + force_c_hat;
            obj.torque_hat = obj.torque + torque_ext + torque_c_hat;                        
            
        end
        
        function sping_evol(obj, t)
            % convert the global variables to local
            Rjw = inv(obj.orientation); 
            p = obj.op_hat - obj.branch_parent.branch_position;
            aux = (Rjw*p');
            I_tilde = Rjw*obj.I_p_w_hat*Rjw' + obj.mass_p_hat*skewm(aux)'*skewm(aux);
            
            torque_tilde = Rjw*obj.torque_hat;
            
            L = chol(I_tilde);
            [x,lambda] = eig(inv(L')*diag(obj.K)*inv(L));
            
%             if ~isreal(x)
%                 sia = 1;
%             end
            U = L\x;
            delta = U'*I_tilde*U;
            
            net_ext_torque = U'*torque_tilde;
            B = diag(obj.beta*lambda + obj.alpha*delta);
            lambda_diag = diag(lambda);
            theta_ = zeros(3,1);
            theta_prime_ = zeros(3,1);
            theta_2prime_ = zeros(3,1);
            for k=1:3
                
                init = obj.init_angles(k);
                init_prime = obj.init_angles_prime(k);
                
                k_spring = lambda_diag(k);
                b = B(k);
                r = 0.5*[-b+sqrt(b^2-4*k_spring) -b-sqrt(b^2-4*k_spring)];
                y_part = net_ext_torque(k)/k_spring;
                coefs = [0 0];
                if isreal(r)
                    if r(1) ~=r(2)                        
                        A_coefs = [1 1;r(1) r(2)];
                        B_coefs = [init - y_part; init_prime];
                        coefs = A_coefs\B_coefs; 
                        
                        theta_(k) = coefs(1) * exp(r(1)*t) + coefs(2) * exp(r(2)*t) + y_part;
                        theta_prime_(k) = r(1)*coefs(1)*exp(r(1)*t) + r(2)*coefs(2)*exp(r(2)*t);
                        theta_2prime_(k) = r(1)^2*coefs(1)*exp(r(1)*t) + r(2)^2*coefs(2)*exp(r(2)*t);
                    else
                        coefs(1) = init - y_part;
                        coefs(2) = init_prime - coefs(1)*r(1);
                        theta_(k) = (coefs(1) + coefs(2)*t)*exp(r(1)*t) + y_part; 
                        theta_prime_(k) = coefs(1)*r(1)*exp(r(1)*t) + coefs(2)*exp(r(1)*t) + r(1)*coefs(2)*exp(r(1)*t)*t;
                        theta_2prime_(k) = r(1)*exp(r(1)*t)*(coefs(1)*r(1) + 2*coefs(2) + r(1)*coefs(2)*t);
                    end
                else
                    real_part = real(r(1));
                    imag_part = imag(r(1));
                    coefs(1) = init - y_part;
                    coefs(2) = (init_prime - real_part*coefs(1))/imag_part; 
                    theta_(k) = exp(t*real_part)*( coefs(1)*cos(imag_part*t) + coefs(2)*sin(imag_part*t) ) + y_part;
                    %variables to calculate the first and second
                    %derivatives
                    cos_1 = exp(t*real_part)*coefs(1)*cos(imag_part*t);
                    sin_1 = exp(t*real_part)*coefs(1)*sin(imag_part*t);
                    cos_2 = exp(t*real_part)*coefs(2)*cos(imag_part*t);
                    sin_2 = exp(t*real_part)*coefs(2)*sin(imag_part*t);
                    
                    theta_prime_(k) = real_part*cos_1 - sin_1*imag_part + real_part*sin_2 + cos_2*imag_part;
                    
                    theta_2prime_(k) = real_part^2*cos_1 - 2*real_part*imag_part*sin_1 - cos_1*imag_part^2 + ...
                        real_part^2*sin_2 + 2*real_part*imag_part*cos_2 - sin_2*imag_part^2;
                end
            end
%             obj.id
            theta = U*theta_;
            obj.theta_all = [obj.theta_all theta];
            theta_prime = U*theta_prime_;
            obj.theta_prime_all = [obj.theta_prime_all theta_prime];
            theta_2prime = U*theta_2prime_;
            th_x = theta(1);
            th_y = theta(2);
            th_z = theta(3);
            Rjc = eul2rotm([th_z, th_y, th_x]);
            
%             obj.init_angles = theta;
%             obj.init_angles_prime = theta_prime;
            
%             plot3(obj.branch_position(1), obj.branch_position(2), obj.branch_position(3), 'xk', 'MarkerSize', 10) %debugging
            obj.orientation_rel = obj.branch_parent.orientation_orig\obj.orientation;
            
%             obj.orientation
            obj.orientation = obj.branch_parent.orientation*obj.orientation_rel_orig*Rjc; %Rwc
%             obj.orientation
            
            obj.orientation_modified = obj.orientation;
            obj.ang_vel = obj.branch_parent.ang_vel + obj.branch_parent.orientation*obj.orientation_rel_orig*theta_prime;
            obj.ang_accel = obj.branch_parent.ang_accel + obj.branch_parent.orientation*obj.orientation_rel_orig*theta_2prime + skewm(obj.branch_parent.ang_vel)*obj.ang_vel;
            
%             d_pj = obj.branch_parent.branch_position_orig;
%             d_cj = obj.branch_position_orig - d_pj;
%             d_pj = obj.branch_parent.orientation\(obj.branch_position_orig - obj.branch_parent.branch_position)';
%             d_pj = obj.branch_parent.orientation_orig\(obj.branch_position_orig - obj.branch_parent.branch_position)';

            d_pj = [norm(obj.branch_position_orig - obj.branch_parent.branch_position_orig) 0 0]';
            d_cj = [0 0 0]';
            
%             aux_1 = obj.branch_parent.orientation*d_pj;
%             aux_2 = obj.orientation * d_cj;            
            aux_1 = obj.orientation*d_pj;
            aux_2 = Rjc*d_cj;
%             aux_1 = obj.branch_parent.orientation*d_pj;            
            
            obj.branch_position = obj.branch_parent.branch_position + aux_1' - aux_2';
            obj.com = (obj.branch_position + obj.branch_parent.branch_position)/2;
            obj.velocity = obj.branch_parent.velocity  + skewm(obj.branch_parent.ang_vel)*aux_1 - skewm(obj.ang_vel)*aux_2;
            obj.accel = obj.branch_parent.accel + (skewm(obj.branch_parent.ang_accel) +  skewm(obj.branch_parent.ang_vel) * skewm(obj.branch_parent.ang_vel))*aux_1 - ...
                (skewm(obj.ang_accel) + skewm(obj.ang_vel) * skewm(obj.ang_vel))*aux_2; %%% check the signs!
            
%             obj.branch_parent.orientation = obj.branch_parent.orientation_orig;  %si va esto en la linea 288 obj.branch_parent.orientation_orig va sin _orig
            
%             plot3(obj.branch_position(1), obj.branch_position(2), obj.branch_position(3), 'xr', 'MarkerSize', 10) %debugging
        end
        
        function reset(obj)
           obj.direction = obj.orig_direction;
           obj.count = 0;
        end
        
        function K = first_guess_k(obj)
            K_ = 1000;
            nu = 0.3; %poisson_ratio see https://www.fpl.fs.fed.us/documnts/fplgtr/fplgtr113/ch04.pdf 
            E = 10e9; %young_mod wood (eye average of values seen 
            % in https://www.engineeringtoolbox.com/timber-mechanical-properties-d_1789.html )
            G = E/(2*(1+nu));
            
            % see https://en.wikipedia.org/wiki/List_of_second_moments_of_area
            In = (1/4)*pi*obj.radius^4; %y local direction
            Ib = (1/4)*pi*obj.radius^4; %z local direction
            It = K_* (1/4)*pi*obj.radius^4; %x local direction , should be greater than the other two 
            %(it is harder to twist along the local x direction)            

            k_t = G * It; %x
            k_b = E * Ib; %y
            k_n = E * In; %z

            K = [k_t k_b k_n];
%             K = obj.orientation*[k_t k_b k_n]';

            
        end
        
        function show(obj, sw, makepickable)
           if ~isempty(obj.branch_parent) 
            h = line([obj.branch_position(1) obj.branch_parent.branch_position(1)], [obj.branch_position(2) obj.branch_parent.branch_position(2)], ...
                [obj.branch_position(3) obj.branch_parent.branch_position(3)], 'LineWidth', sw);
            if ~makepickable
                set(h, 'PickableParts', 'none')
            else
                set(h, 'PickableParts', 'visible')
            end
           end
        end
        
        function show_cylinders(obj, new_radius, is_pruned)
           obj.radius = new_radius;
           if ~isempty(obj.branch_parent)
               if ~is_pruned
               obj.K = obj.first_guess_k./[1 0.05 0.05]; % the division is debugging [1 2 2], [1 10 2], [1 10 9.3], [100 100 5] [1000 50 5] [1000 30 3], [1000 20 2] -> z_4, y_1, [1000 3 2] -> x_3,  [1000 .015 0.5] -> x_12
               else
                obj.K = obj.first_guess_k./[1 5 5]; 
               end
                [x, y, z] = obj.cylinder_x_axis(new_radius);
               
               
               xyz1_ = obj.orientation*[x(1,:); y(1,:); z(1,:)];
%                xyz1_ = obj.orientation_modified*[x(1,:); y(1,:); z(1,:)];
               x1 = xyz1_(1,:); y1 = xyz1_(2,:); z1 = xyz1_(3,:);
               
               xyz2_ = obj.orientation*[x(2,:); y(2,:); z(2,:)];
%                xyz2_ = obj.orientation_modified*[x(2,:); y(2,:); z(2,:)];
               x2 = xyz2_(1,:); y2 = xyz2_(2,:); z2 = xyz2_(3,:);
               
               x_new = [x1;x2];
               y_new = [y1;y2];
               z_new = [z1;z2];
               surf(x_new + obj.branch_parent.branch_position(1), y_new + obj.branch_parent.branch_position(2), z_new + obj.branch_parent.branch_position(3))

           end
        end
        
        function [x_new, y_new, z_new] = cylinder_x_axis(obj, radius)
            [x, y, z] = cylinder(radius);
            z(2,:) = obj.effective_len;
            aux = roty(90)*[x(1,:); y(1,:); z(1,:)];
            x1 = aux(1,:); y1 = aux(2,:); z1 = aux(3,:);
            
            aux = roty(90)*[x(2,:); y(2,:); z(2,:)];
            x2 = aux(1,:); y2 = aux(2,:); z2 = aux(3,:);
            x_new = [x1;x2];
            y_new = [y1;y2];
            z_new = [z1;z2];           
        end
        
    end
    
end
