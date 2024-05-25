% Load the remote API
sim = remApi('remoteApi');
sim.simxFinish(-1); % Close any existing connections

% Connect to CoppeliaSim
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
if clientID > -1
    disp('Connected to remote API server');
    
    % Get object handles
    [~, left_motor] = sim.simxGetObjectHandle(clientID, 'motor_left', sim.simx_opmode_blocking);
    [~, right_motor] = sim.simxGetObjectHandle(clientID, 'motor_right', sim.simx_opmode_blocking);
    [~, north_sensor] = sim.simxGetObjectHandle(clientID, 'front_prox', sim.simx_opmode_blocking);
    [~, right_sensor] = sim.simxGetObjectHandle(clientID, 'front_right', sim.simx_opmode_blocking);
    [error_code, left_sensor] = sim.simxGetObjectHandle(clientID, 'front_left', sim.simx_opmode_blocking);

    % Define control functions
    go_straight = @(speed) sim.simxSetJointTargetVelocity(clientID, left_motor, speed, sim.simx_opmode_oneshot);
    rotate_right = @(speed) sim.simxSetJointTargetVelocity(clientID, left_motor, speed, sim.simx_opmode_oneshot);
    rotate_left = @(speed) sim.simxSetJointTargetVelocity(clientID, left_motor, -speed, sim.simx_opmode_oneshot);

    % Start streaming sensor data
    sim.simxReadProximitySensor(clientID, north_sensor, sim.simx_opmode_streaming);
    sim.simxReadProximitySensor(clientID, right_sensor, sim.simx_opmode_streaming);
    sim.simxReadProximitySensor(clientID, left_sensor, sim.simx_opmode_streaming);

    % Example object position retrieval
    [returnCode, robothandle1] = sim.simxGetObjectHandle(clientID, 'GPS', sim.simx_opmode_blocking);
    if returnCode == sim.simx_return_ok
        [~, position] = sim.simxGetObjectPosition(clientID, robothandle1, -1, sim.simx_opmode_streaming);
        disp(position);
    else
        disp('Failed to get GPS handle');
    end

    % Define additional handles and variables
    [~, robothandle] = sim.simxGetObjectHandle(clientID, 'Robotpose', sim.simx_opmode_blocking);
    [~, robothandle2] = sim.simxGetObjectHandle(clientID, 'GyroSensor', sim.simx_opmode_blocking);
    [~, ~] = sim.simxGetObjectOrientation(clientID, robothandle2, -1, sim.simx_opmode_streaming);
    stored_vector = 0;
    sum_Ki = 0;
    goal = [0, 0];
    previous_error_angle = Initial_read(clientID, robothandle1, robothandle2, goal);
    while true
        [isReached, previous_error_angle, sum_Ki, stored_vector] = move_to_goal(clientID, left_motor, right_motor, robothandle1, robothandle2, goal, previous_error_angle, sum_Ki, stored_vector);
        if isReached
            break;
        end
        pause(0.05);
    end
    plot(stored_vector);
    xlabel('Time');
    ylabel('Error Angle');
    title('Error Angle vs Time');
    disp('Program ended');
else
    disp('Failed connecting to remote API server');
end

function [state, distance] = read_sensor(clientID, sensor)
    global sim;
    [res, state, point, ~, ~] = sim.simxReadProximitySensor(clientID, sensor, sim.simx_opmode_buffer);
    distance = norm(point);
    if res ~= sim.simx_return_ok
        disp('Failed reading proximity sensor');
    end
end

function angle = my_atan(vector_x, vector_y)
    if vector_x == 0
        if vector_y > 0
            angle = pi/2;
        elseif vector_y < 0
            angle = 1.5*pi;
        elseif vector_y == 0
            angle = 0;
        end
    elseif vector_y == 0
        if vector_x > 0
            angle = 0;
        elseif vector_x < 0
            angle = pi;
        elseif vector_x == 0
            angle = 0;
        end
    elseif vector_x * vector_y > 0
        if vector_x > 0
            angle = atan(vector_y / vector_x);
        elseif vector_x < 0
            angle = pi + atan(vector_y / vector_x);
        end
    elseif vector_x * vector_y < 0
        if vector_x > 0
            angle = 2*pi + atan(vector_y / vector_x);
        elseif vector_x < 0
            angle = pi + atan(vector_y / vector_x);
        end
    end
end

function errorAngle = caculate_diff_angle(desired_orientation, curr_orientation)
    tempt = desired_orientation - curr_orientation;
    t = 2*pi - abs(tempt);
    if abs(tempt) <= pi
        errorAngle = tempt;
    else
        if tempt <= 0
            errorAngle = t;
        else
            errorAngle = -t;
        end
    end
end

function errorAngle = Initial_read(clientID, robothandle1, robothandle2, goal)
    global sim;
    [~, position] = sim.simxGetObjectPosition(clientID, robothandle1, -1, sim.simx_opmode_buffer);
    [~, orientation] = sim.simxGetObjectOrientation(clientID, robothandle2, -1, sim.simx_opmode_buffer);
    xy_position = position(1:2);
    V_m2g = goal - xy_position;
    desired_orientation = my_atan(V_m2g(1), V_m2g(2));
    curr_orientation = my_atan(orientation(1), orientation(2));
    curr_orientation = curr_orientation - pi/2;
    if curr_orientation < 0
        curr_orientation = curr_orientation + 2*pi;
    end
    errorAngle = caculate_diff_angle(desired_orientation, curr_orientation);
end

function [isReached, previous_error_angle, sum_Ki, stored_vector] = move_to_goal(clientID, left_motor, right_motor, robothandle1, robothandle2, goal, previous_error_angle, sum_Ki, stored_vector)
    global sim;
    dm2_g = 0.05;
    am2_g = 70;
    Kp = 5;
    Kd = 0;
    Ki = 0.2;
    MAX_ANGULAR_VECTOR = 50;
    R = 0.03;
    L = 0.1665;
    MAX_SPEED = 1;
    dt = 55/1000;
    [~, position] = sim.simxGetObjectPosition(clientID, robothandle1, -1, sim.simx_opmode_buffer);
    xy_position = position(1:2);
    [~, orientation] = sim.simxGetObjectOrientation(clientID, robothandle2, -1, sim.simx_opmode_buffer);
    V_m2g = goal - xy_position;
    desired_orientation = my_atan(V_m2g(1), V_m2g(2));
    curr_orientation = my_atan(orientation(1), orientation(2));
    curr_orientation = curr_orientation - pi/2;
    if curr_orientation < 0
        curr_orientation = curr_orientation + 2*pi;
    end
    dist_goal = norm(V_m2g);
    if dist_goal ~= 0
        V_m2g = am2_g * V_m2g / dist_goal;
    end
    if dist_goal < dm2_g
        sim.simxSetJointTargetVelocity(clientID, left_motor, 0, sim.simx_opmode_oneshot);
        sim.simxSetJointTargetVelocity(clientID, right_motor, 0, sim.simx_opmode_oneshot);
        disp('End');
        isReached = true;
        return;
    end
    errorAngle = caculate_diff_angle(desired_orientation, curr_orientation);
    speed = dist_goal / 4;
    if speed >= MAX_SPEED
        speed = MAX_SPEED;
    end
    if errorAngle >= MAX_ANGULAR_VECTOR
        errorAngle = MAX_ANGULAR_VECTOR;
    end
    t = (Kd * (errorAngle - previous_error_angle) / dt);
    sum_Ki = sum_Ki + (errorAngle + previous_error_angle) * dt / 2;
    Wr = (speed / 2 + (Kp * errorAngle + t + Ki * sum_Ki) * L / 2) / R;
    Wl = (speed / 2 - (Kp * errorAngle + t + Ki * sum_Ki) * L / 2) / R;
    sim.simxSetJointTargetVelocity(clientID, left_motor, Wl, sim.simx_opmode_oneshot);
    sim.simxSetJointTargetVelocity(clientID, right_motor, Wr, sim.simx_opmode_oneshot);
    stored_vector = [stored_vector, errorAngle];
    isReached = false;
end