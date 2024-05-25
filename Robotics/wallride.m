
sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
if (clientID > -1)
    disp('Connected to remote Api server');
    VL = 0.5;
    VR = 0.5;
    [~, robot]= sim.simxGetObjectHandle(clientID,'robot',sim.simx_opmode_blocking);
    [~, left_motor] = sim.simxGetObjectHandle(clientID, 'motor_left', sim.simx_opmode_blocking);
    [~, right_motor] = sim.simxGetObjectHandle(clientID, 'motor_right', sim.simx_opmode_blocking);
    [~, front_prox_handle] = sim.simxGetObjectHandle(clientID, 'front_prox', sim.simx_opmode_blocking);
    [~, front_right_handle] = sim.simxGetObjectHandle(clientID,'front_right',sim.simx_opmode_blocking);
    [~, rear_right_handle] = sim.simxGetObjectHandle(clientID,'rear_right',sim.simx_opmode_blocking);
    [~, front_left_handle] = sim.simxGetObjectHandle(clientID,'front_left',sim.simx_opmode_blocking);
    [~, rear_left_handle] = sim.simxGetObjectHandle(clientID,'rear_left',sim.simx_opmode_blocking);

    [~] = sim.simxSetJointTargetVelocity(clientID, left_motor, VL, sim.simx_opmode_streaming);
    [~] = sim.simxSetJointTargetVelocity(clientID, right_motor, VR, sim.simx_opmode_streaming);
       
    sensorHandle = [front_prox_handle, front_right_handle, rear_right_handle, front_left_handle, rear_left_handle];

   for i = 1:length(sensorHandle)
       sim.simxReadProximitySensor(clientID, sensorHandle(i), sim.simx_opmode_streaming);
   end

   while(true)
       f = 0.5;
       %%
       output = readSensors(clientID,sim, sensorHandle);
       detectionStates = cell2mat(output(1));
       distances = cell2mat(output(2));
       %%
       if (detectionStates(1) == 1)
           f = distances(1);
       end
       
       if (detectionStates(2) == 1 && detectionStates(3) == 1)
           average = (distances(2) + distances(3))/2;
           dif = distances(2) - distances(3);
       else
           average = 0;
           dif = 0;
       end
       
       if (f > 0.25 && detectionStates(2) == 1 && detectionStates(3) == 1)
           if (average > 0.12)
                [~] = sim.simxSetJointTargetVelocity(clientID, left_motor, VL, sim.simx_opmode_streaming);
                [~] = sim.simxSetJointTargetVelocity(clientID, right_motor, 0, sim.simx_opmode_streaming);
           elseif (average < 0.07)
                [~] = sim.simxSetJointTargetVelocity(clientID, left_motor, 0, sim.simx_opmode_streaming);
                [~] = sim.simxSetJointTargetVelocity(clientID, right_motor, VR, sim.simx_opmode_streaming);
           elseif (average > 0.07 && average < 0.12)
               if(dif > 0.025)
                    [~] = sim.simxSetJointTargetVelocity(clientID, left_motor, 0, sim.simx_opmode_streaming);
                    [~] = sim.simxSetJointTargetVelocity(clientID, right_motor, VR, sim.simx_opmode_streaming);
               elseif(dif < -0.025)
                    [~] = sim.simxSetJointTargetVelocity(clientID, left_motor, VL, sim.simx_opmode_streaming);
                    [~] = sim.simxSetJointTargetVelocity(clientID, right_motor, 0, sim.simx_opmode_streaming);
               else
                    [~] = sim.simxSetJointTargetVelocity(clientID, left_motor, VL, sim.simx_opmode_streaming);
                    [~] = sim.simxSetJointTargetVelocity(clientID, right_motor, VR, sim.simx_opmode_streaming);
               end
           end
       elseif(f > 0.25 && detectionStates(2) == 0 && detectionStates(3) == 1)
           [~] = sim.simxSetJointTargetVelocity(clientID, left_motor, VL, sim.simx_opmode_streaming);
           [~] = sim.simxSetJointTargetVelocity(clientID, right_motor, 0, sim.simx_opmode_streaming);
           %% 
       elseif(f < 0.25)
           [~] = sim.simxSetJointTargetVelocity(clientID, left_motor, 0, sim.simx_opmode_streaming);
           [~] = sim.simxSetJointTargetVelocity(clientID, right_motor, VR, sim.simx_opmode_streaming);
       end
   end
     

else 
    disp('Can not connect to Api server');
end

sim.delete();
disp('Program ended');

function output = readSensors(clientID, sim, sensorHandle)
   
    detectionStates = zeros(1, length(sensorHandle));
    dis = zeros(1, length(sensorHandle));

for i = 1:length(sensorHandle)
    [~,detectionState, dectectedPoint,~,~] = sim.simxReadProximitySensor(clientID, sensorHandle(i), sim.simx_opmode_buffer);
    if detectionState == 1
        detectionStates(i) = detectionState;
        dis(i) = dectectedPoint(3);
    end
end
    output = {detectionStates,dis};
end