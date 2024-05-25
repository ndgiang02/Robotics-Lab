classdef GoToGoal
    %% GOTOGOAL steers the robot towards a goal with a constant velocity using PID
    %
    % Properties:
    %   none
    %
    % Methods:
    %   execute - Computes the left and right wheel speeds for go-to-goal.

    properties
        %% PROPERTIES
        
        % memory banks
        E_k
        e_k_1
        
        % gains
        Kp
        Ki
        Kd
        
        % plot support
        p
        
        % Offset
        offset
    end
    
    properties (Constant)
        % I/O
        inputs = struct('x_g', 0, 'y_g', 0, 'v', 0);
        outputs = struct('v', 0, 'w', 0);
    end
    
    methods
        %% METHODS
        
        function obj = GoToGoal(offset)
           
            
            % initialize memory banks
            obj.Kp = 4;
            obj.Ki = 0.01;
            obj.Kd = 0.01;
                        
            % errors
            obj.E_k = 0;
            obj.e_k_1 = 0;
            
            % plot support
            obj.p = [];
            
            % Offset
            obj.offset = offset;
        end
        
        function outputs = execute(obj, robot, state_estimate, inputs, dt)
            %% EXECUTE Computes the left and right wheel speeds for go-to-goal.
            %   [v, w] = execute(obj, robot, x_g, y_g, v) will compute the
            %   necessary linear and angular speeds that will steer the robot
            %   to the goal location (x_g, y_g) with a constant linear velocity
            %   of v.
            %
            %   See also controller/execute
            
            % Retrieve the (relative) goal location
            x_g = inputs.x_g; 
            y_g = inputs.y_g;
            
            % Get estimate of current pose
            [x, y, theta] = state_estimate.unpack();
            
            % Compute vector from robot to goal
            V_m2g = [x_g - x; y_g - y];
            dist2goal = norm(V_m2g);
            
            % Check if goal is reached
            if dist2goal < obj.offset
                % Stop the robot if goal is reached
                outputs.v = 0;
                outputs.w = 0;
                disp('Goal reached!');
                return;  % Exit function
            end
            
            % Compute heading (angle) to the goal
            theta_g = atan2(V_m2g(2), V_m2g(1));
            
            % Calculate heading error
            e_k = theta_g - theta;
            e_k = atan2(sin(e_k), cos(e_k));
            
            % Calculate PID for the steering angle
            e_P = e_k;
            e_I = obj.E_k + e_k * dt;
            e_D = (e_k - obj.e_k_1) / dt;
            w = obj.Kp * e_P + obj.Ki * e_I + obj.Kd * e_D;
            
            % Save errors for next time step
            obj.E_k = e_I;
            obj.e_k_1 = e_k;
            
            % Plot
            obj.p.plot_2d_ref(dt, atan2(sin(theta), cos(theta)), theta_g, 'r');
            
            % Set linear velocity
            outputs.v = inputs.v;
            % Set angular velocity
            outputs.w = w;
        end
        
        function reset(obj)
            % Reset accumulated and previous error
            obj.E_k = 0;
            obj.e_k_1 = 0;
        end
    end
    
end
