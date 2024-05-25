vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
L = 0.1665;
R = 1;
if (clientID > -1)
    disp('Connected to remote Api server');

    [~,left_Motor] = vrep.simxGetObjectHandle(clientID, 'motor_left', vrep.simx_opmode_blocking);
    [~,right_Motor] = vrep.simxGetObjectHandle(clientID, 'motor_right', vrep.simx_opmode_blocking);

       
    %read sensor
    [~, front_sensor] = vrep.simxGetObjectHandle(clientID, 'front_prox', vrep.simx_opmode_blocking);

    vrep.simxReadProximitySensor(clientID, front_sensor, vrep.simx_opmode_streaming);

    while(true)
        [~, state, point, ~, ~] = vrep.simxReadProximitySensor(clientID, front_sensor, vrep.simx_opmode_buffer)
        %disp(point(3));
        pause(0.5);
        if (point(3) > 0.1 && point(3) < 0.3)
            [~] = vrep.simxSetJointTargetVelocity(clientID, left_Motor, -1, vrep.simx_opmode_oneshot);
            [~] = vrep.simxSetJointTargetVelocity(clientID, right_Motor, 1, vrep.simx_opmode_oneshot);
        else
            [~] = vrep.simxSetJointTargetVelocity(clientID, left_Motor, 1, vrep.simx_opmode_oneshot);
            [~] = vrep.simxSetJointTargetVelocity(clientID, right_Motor, 1, vrep.simx_opmode_oneshot);
        end
    end
else 
    disp('Can not connect to Api server');
end