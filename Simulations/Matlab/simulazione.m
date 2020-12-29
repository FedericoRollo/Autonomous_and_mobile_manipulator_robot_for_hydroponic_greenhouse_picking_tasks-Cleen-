% Make sure to have the server side running in CoppeliaSim: 
% in a child script of a CoppeliaSim scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

function simulazione()
    clc; %close all;
    disp('>>Simulation started');
    % sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5); 
    
    
    if (clientID>-1)
        disp('>>Connected to remote API server');

        
        %Arm known and generally used joint positions
        arm_home_position = [0 90 90 0 -90 0];
        flower_release_position = [180 90 -20 0 -50 0];
        flower_pointing_position =[-90 110 30 0 -85 0];
        global j_p;
        [k_q, j_q, j_p] = ArmKinematics();
        


        %number of plants
        number_plants = 8; 
        current_flower = 0;
        platform_home_position = number_plants+1;
        %kinect vision system object handles
        [res,kinect_rgb] = sim.simxGetObjectHandle(clientID,'kinect_rgb',sim.simx_opmode_blocking);
        [res,kinect_depth] = sim.simxGetObjectHandle(clientID,'kinect_depth',sim.simx_opmode_blocking);

            
        %Begin harvesting algorithm
        for current_plant = 1:number_plants
            %move robot on next plant
            fprintf('>>\n>>Moving platform towards plant %d\n',current_plant);
            moveRobotOnPlant(sim,clientID,current_plant)
            %position the end effector in flower pointing position
            fprintf('>>Moving manupulator arm in plant %d pointing position \n',current_plant);
            moveArmInTargetJointPosition(sim, clientID, flower_pointing_position)
%             pause(1.5)
            %get RGB image for flower identification
            [res,resolution,img] = sim.simxGetVisionSensorImage2(clientID,kinect_rgb,0,sim.simx_opmode_blocking);
            
            %K-mean algorithm for searching flowers on the plant
            fprintf('>>Searching for flowers on plant %d with K-mean algorithm\n',current_plant);
            thereIsFlower = searchForFlowersOnPlant(img);
            
            %If there are flower start the localization and picking
            %sequence otherwise continue to next plant
            if thereIsFlower
                fprintf('>>There are flowers on plant %d\n',current_plant)
                
                %APPROACHING MOVEMENT
                fprintf('>>Plant %d approaching movement\n',current_plant)
                approachFlower(sim,clientID,1);
                
                %FLOWERS LOCALIZATION
                %get another RGB image to localize flowers
                [res,resolution,img_rgb] = sim.simxGetVisionSensorImage2(clientID,kinect_rgb,0,sim.simx_opmode_blocking);
                
                %image segmentation to extract flowers centroids
                fprintf('>>Research and image segmentation of flower stems on plant %d\n>>stem centroids computation\n',current_plant);
                centroids = searchForStems(img_rgb);
                
                %extract the depth informations from the kinect camera
                fprintf('>>Flower localization on plant %d\n',current_plant);
                [res,resolution,img_depth] = sim.simxGetVisionSensorDepthBuffer2(clientID,kinect_depth,sim.simx_opmode_blocking);
                
                %'FOR' LOOP FOR FLOWER PICKING
                for i = 1:size(centroids,1)
                    flower_string = sprintf('%s%d','Fiore',current_flower);
                    [res,curr_flower] = sim.simxGetObjectHandle(clientID,flower_string,sim.simx_opmode_blocking);
                    [res,flower_position] = sim.simxGetObjectPosition(clientID,curr_flower,-1,sim.simx_opmode_blocking);
                    
                    moveArmInTargetJointPosition(sim, clientID, flower_pointing_position)
%                     pause(3);
                    %extract the centroid pixel position
                    image_x = round(centroids(i,1));
                    image_y = round(centroids(i,2));
                    
                    f = 0.00173667;
                    fprintf('>>Centroid %d   x:%f   y:%f\n',i,image_x,image_y)
                    
                    %compute real coordinates of the flower
                    z = img_depth(image_x,image_y); 
                    x = double(image_x-resolution(1)/2)*f*z;
                    y = double(image_y-resolution(2)/2)*f*z;
                    fprintf('>>Flower coordinates in world reference frame: x: %d   y:%d   z:%d\n',double(flower_position(1)),double(flower_position(2)),double(flower_position(3)));
%                   target_position = [x y z];

                    %move arm on target position

                    moveArm(sim,clientID,double(curr_flower));
                    connectFlower(sim,clientID,curr_flower,1);
                    
                    sim.simxSetIntegerSignal(clientID,'BaxterGripper_close',1,sim.simx_opmode_oneshot_wait);
                    matlabWaitVrep(sim,clientID);
                    
%                   %RELEASE FLOWER IN THE BOX
                    moveArmInTargetJointPosition(sim, clientID, flower_release_position)
                    sim.simxSetIntegerSignal(clientID,'BaxterGripper_close',0,sim.simx_opmode_oneshot_wait);
                    connectFlower(sim,clientID,curr_flower,0);
                    matlabWaitVrep(sim,clientID);
                    
                    current_flower = current_flower + 1;
                end

                %arm return to home position
                moveArmInTargetJointPosition(sim, clientID, arm_home_position)
%                 pause(1.5);
               
                %RETURN ON PATH
                disp('>>Return on main path')
                approachFlower(sim,clientID,0);
            else %Thera are Not flowers on the plant so continue to the next plant or to platform home position
                fprintf('>>There are NOT flowers on plant %d \n',current_plant)
                  %arm return to home position
                moveArmInTargetJointPosition(sim, clientID, arm_home_position)

            end
        end

        %Move the robot in home position
        fprintf('>>Harvest finished\n>>Return to docking station')
        moveRobotOnPlant(sim,clientID,platform_home_position);
        
        
        % stop the simulation
        sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);

        % Now close the connection to Vrep    
        sim.simxFinish(clientID);
        
    else
        disp('Failed connecting to remote API server');
    end
    sim.delete(); % call the destructor!
    
    disp('Simulation ended');
end
function connectFlower(sim,clientID,flower,connect)
    
    [res,connector] = sim.simxGetObjectHandle(clientID,'BaxterGripper_attachPoint',sim.simx_opmode_blocking);
    [res,box] = sim.simxGetObjectHandle(clientID,'CollectingBox',sim.simx_opmode_blocking);
    
    if connect==1 %connect flower
        sim.simxSetObjectParent(clientID,flower,connector,1,sim.simx_opmode_blocking);
        sim.simxSetObjectIntParameter(clientID,flower,3004,0,sim.simx_opmode_blocking);
        sim.simxSetObjectIntParameter(clientID,flower,3003,0,sim.simx_opmode_blocking);
        disp('>>Flower connected to end-effector');
    else %detach flower
        disp('>>Flower detached from end-effector'); 
        sim.simxSetObjectIntParameter(clientID,flower,3004,1,sim.simx_opmode_blocking);
        res = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript,'resetObjectDyn',[flower],[],'',[],sim.simx_opmode_oneshot);
        pause(0.7);
        sim.simxSetObjectParent(clientID,flower,-1,0,sim.simx_opmode_blocking);
        pause(0.8)
        sim.simxSetObjectParent(clientID,flower,box,1,sim.simx_opmode_blocking);
    end
    
end

function approachFlower(sim,clientID, approach)
     sim.simxSynchronous(clientID,true); 
     
     [res,car] = sim.simxGetObjectHandle(clientID,'MecanumCar',sim.simx_opmode_blocking);
     [res,initial_position] = sim.simxGetObjectPosition(clientID,car,-1,sim.simx_opmode_oneshot_wait);
     speed=5;
     if approach==1 
         wheelSpeed=[-speed -speed speed speed];
     else
         wheelSpeed=[speed speed -speed -speed];
     end
     spostamento = 0;
     while spostamento < 0.235
         [res] = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript,'rotateWheels',wheelSpeed,[],'',[],sim.simx_opmode_blocking);
         sim.simxSynchronousTrigger(clientID);
         [res,current_position] = sim.simxGetObjectPosition(clientID,car,-1,sim.simx_opmode_oneshot_wait);
         spostamento = norm(initial_position(1) - current_position(1));
     end
     [res] = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript,'rotateWheels',[0 0 0 0],[],'',[],sim.simx_opmode_blocking);
     [res] = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript,'rotateWheels',[0 0 0 0],[],'',[],sim.simx_opmode_blocking);
     sim.simxSynchronousTrigger(clientID);
    sim.simxGetPingTime(clientID);
    sim.simxSynchronous(clientID,false);   
end

function moveArmInTargetJointPosition(sim, clientID, targetPos)
    
    [res] = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript, 'moveRobotToJointPosition',targetPos,[],'',[],sim.simx_opmode_blocking);
    joints=[-1 -1 -1 -1 -1 -1];
    vel = [-1 -1 -1 -1 -1 -1];
    totvel=0;
    for i = 1:6 
       joint_string = sprintf('%s%d','joint',i);
       [res,joints(i)]=sim.simxGetObjectHandle(clientID,joint_string,sim.simx_opmode_blocking);
       [res,vel(i)] = sim.simxGetObjectFloatParameter(clientID,joints(i),2012,sim.simx_opmode_blocking);
       totvel=totvel+norm(vel(i));
    end
    
    while totvel>0.01
        totvel=0;
        for i = 1:6 
            [res,vel(i)] = sim.simxGetObjectFloatParameter(clientID,joints(i),2012,sim.simx_opmode_blocking);
            totvel=totvel+norm(vel(i));
        end
        pause(0.1)
    end
    
%     matlabWaitVrep(sim,clientID);
end

function moveRobotOnPlant(sim, clientID, n)
    if(n==1)
        [res] = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript, 'startTraj',0,[],'',[],sim.simx_opmode_blocking);
        matlabWaitVrep(sim,clientID)%pause(16)
        [res] = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript(1), 'startTraj',1,[],'',[],sim.simx_opmode_blocking);
        matlabWaitVrep(sim,clientID)%pause(6)
    elseif(n>1 && n<5)
        [res] = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript(1), 'startTraj',n,[],'',[],sim.simx_opmode_blocking);
        matlabWaitVrep(sim,clientID)
    elseif(n==5)
        [res] = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript(1), 'startTraj',5,[],'',[],sim.simx_opmode_blocking);
        matlabWaitVrep(sim,clientID)
        [res] = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript(1), 'startTraj',6,[],'',[],sim.simx_opmode_blocking);
        matlabWaitVrep(sim,clientID)
        [res] = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript(1), 'startTraj',7,[],'',[],sim.simx_opmode_blocking);
        matlabWaitVrep(sim,clientID)
    elseif(n>5 && n<9)
        [res] = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript(1), 'startTraj',n+2,[],'',[],sim.simx_opmode_blocking);
        matlabWaitVrep(sim,clientID)
    elseif(n==9)% return to the docking station
        [res] = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript(1), 'startTraj',11,[],'',[],sim.simx_opmode_blocking);
        matlabWaitVrep(sim,clientID)
        [res] = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript(1), 'startTraj',12,[],'',[],sim.simx_opmode_blocking);
        matlabWaitVrep(sim,clientID)
    else
        disp('plant number not available')
    end
end

function matlabWaitVrep(sim,clientID)
        
        pause(1.5);
        [res,matlabStop] = sim.simxGetIntegerSignal(clientID,'matlabWait',sim.simx_opmode_blocking);
        
    while (matlabStop==1) 
        pause(0.25)
        [res,matlabStop] = sim.simxGetIntegerSignal(clientID,'matlabWait',sim.simx_opmode_blocking);
%         disp('wait');
    end
end

function moveArm(sim,clientID,curr_flower)
    sim.simxSynchronous(clientID,true);
    global j_p
    
    jointHandle = {-1,-1,-1,-1,-1,-1};
    current_q = [-1 -1 -1 -1 -1 -1];
%     [res,target_position] = sim.simxGetObjectPosition(clientID,curr_flower,-1,sim.simx_opmode_blocking);

    [res,ee_orient] = sim.simxGetObjectHandle(clientID,'BaxterGripper',sim.simx_opmode_blocking);
    
     [res,flower_child]=sim.simxGetObjectChild(clientID,curr_flower,0,sim.simx_opmode_blocking);
     [res,target_position] = sim.simxGetObjectPosition(clientID,flower_child,-1,sim.simx_opmode_blocking);

     [res,flower_orientation] = sim.simxGetObjectOrientation(clientID,flower_child,ee_orient, sim.simx_opmode_blocking);

    
    syms q1 q2 q3 q4 q5 q6

    for i = 1:6
        joint = sprintf('%s%d','joint',i);
        [res,jointHandle{i}] = sim.simxGetObjectHandle(clientID,joint,sim.simx_opmode_blocking);
        sim.simxSetObjectIntParameter(clientID,jointHandle{i},2001,0,sim.simx_opmode_blocking);
        current_q(i) = sim.simxGetJointPosition(clientID,jointHandle{i},sim.simx_opmode_blocking);
    end
    
    [res,joint_angle] = sim.simxGetJointPosition(clientID,jointHandle{6},sim.simx_opmode_blocking);


    [res,ee] = sim.simxGetObjectHandle(clientID,'BaxterGripper_attachPoint',sim.simx_opmode_blocking);
    [res,arm] = sim.simxGetObjectHandle(clientID,'MyRoboticArm',sim.simx_opmode_blocking);
            
    [res,ee_position] = sim.simxGetObjectPosition(clientID,ee,-1, sim.simx_opmode_blocking);
    [res,robot_orientation] = sim.simxGetObjectOrientation(clientID,arm,-1, sim.simx_opmode_blocking);
    z_angle = robot_orientation(3);
    rotation_matrix = [cos(z_angle) sin(z_angle) 0; -sin(z_angle) cos(z_angle) 0; 0 0 1];

    
    error = rotation_matrix*(target_position-ee_position)';
    angle_error = (flower_orientation(3) - joint_angle);
    
    k=2*eye(3);
    k_angle = 1;
    while norm(error)>0.012
        
        if(norm(error)<0.1)
            k = 10*eye(3);
%         elseif(norm(error)<0.01)
%             k = 100*eye(3);
        end
       
        for i = 1:6
            [res,current_q(i)] = sim.simxGetJointPosition(clientID,jointHandle{i},sim.simx_opmode_oneshot);
        end

        j_p_curr = double(subs(j_p,[q1 q2 q3 q4 q5 q6], current_q));
        j_pinv = pinv(j_p_curr);
        q_dot = j_pinv*k*error;
        
        res = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript,'setJointVel',[],[q_dot'],'',[],sim.simx_opmode_oneshot);
        res = sim.simxSetJointTargetVelocity(clientID,jointHandle{6},2*angle_error,sim.simx_opmode_blocking);

        
        sim.simxSynchronousTrigger(clientID);

        [res,ee_position] = sim.simxGetObjectPosition(clientID,ee,-1, sim.simx_opmode_blocking);
        [res,joint_angle] = sim.simxGetJointPosition(clientID,jointHandle{6},sim.simx_opmode_blocking);

        error = rotation_matrix*(target_position-ee_position)';
        angle_error = (flower_orientation(3) - joint_angle);
    end

    q_end = [0 0 0 0 0 0];
    res = sim.simxCallScriptFunction(clientID,'MecanumCar',sim.sim_scripttype_childscript,'setJointVel',[],[q_end'],'',[],sim.simx_opmode_oneshot);
    sim.simxSynchronousTrigger(clientID);
    
  
    sim.simxSynchronous(clientID,false);
    
    
      for i = 1:6
         [res,pos] = sim.simxGetJointPosition(clientID,jointHandle{i},sim.simx_opmode_blocking);
         sim.simxSetJointTargetPosition(clientID,jointHandle{i},pos,sim.simx_opmode_blocking);
         sim.simxSetObjectIntParameter(clientID,jointHandle{i},2001,1,sim.simx_opmode_blocking);
      end
end
