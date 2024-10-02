close all; clear all; clc; format compact; 
addpath('hebi','symbolic_functions');

%% Hebi Setup
% joy = vrjoystick(1) ;

% move group setup
% first set of values is for original arm
% second set of values is for new arm
serials_move = {    
%                      'x-80663' %rotation base
%                      'x-81208' %shoulder 1a
%                      'x-81128' %shoulder 1b
%                      'x-80978' %shoulder 2
%                      'x-80760' %intermediate
%                      'x-81034' %leveler


                     'x-81068' %rotation base
                     'x-80738' %shoulder 1a
                     'x-81022' %shoulder 1b
                     'x-81069' %shoulder 2
                     'x-81036' %intermediate
                     'x-81206' %leveler 
                };
group_move = HebiLookup.newGroupFromSerialNumbers(serials_move);
cmd_move = CommandStruct();
fbk_move = getNextFeedback(group_move);

%orig arm
%serials_gripper = {'x-01075'}; %gripper

%second arm
serials_gripper = {'x-01129'}; %gripper

group_gripper = HebiLookup.newGroupFromSerialNumbers(serials_gripper);
cmd_gripper = CommandStruct();
fbk_gripper = getNextFeedback(group_gripper);


fprintf('hebi setup\n')

%% position determination
q=fbk_move.position; 
x=func_sym_arm_kinematics_3D(q');

% Start/Safety Position
motor_pos = [pi/2 -pi/3 pi/3 -pi/2 -pi/3 (-pi/2-(pi/6-pi/6))];

reset_position = 1;
q_lim = 20*pi/180;
max_speed = [25*pi/180 50*pi/180 50*pi/180 50*pi/180 50*pi/180 50*pi/180];
fprintf('safety\n')
tic
t = toc;
while toc-t < 6
    fbk_move = getNextFeedback(group_move) ;
    q = fbk_move.position ;
    qdot = motor_pos-q; 
    cmd_move.velocity = qdot.*max_speed ;
    cmd_gripper.position = 1;
    group_move.send(cmd_move);
    group_gripper.send(cmd_gripper);
    reset_position = norm( q ) ; 
    pause (.01)
end
fprintf('start\n')
tstart = toc;

%% COM Proof-of-Concept

%COM without round
COM_masses = [0.2155; 0.2889]; % [m], COM along [first link; second link]
mass = [2.6720;1.2990]; % [kg], mass for [first link; second link]

%COM with round
COM_masses_round = [0.2155; 0.3113]; % [m], COM along [first link; second link]
mass_round = [2.6720; 2.8490]; % [kg], mass for [first link; second link]

g = 9.81; %gravity variable
speed_max = 100; % [rad/s] max angular velocity, set high so it does not affect subsequent code

%movement setup
sumError = [0];
errorLast = [0];

t = toc;

%% Fixed points
points = [0.36,-0.16;
          0.45,-0.25;
          0.47,-0.21;
          0.31,-0.21]; %these points should be adjusted

%% Stays turned on (fixed destination)
destination = [0.47, 0.39;
               0.47,0.21;
               0.31,0.21;
               0.31,0.39;];
points
for i = 1:size(points,1)
    %% Go to round
    fprintf('Go to round')
    z = points(i,:)'; %takes first ith row
  
    z(2) = z(2)-0.045; %may have to adjust this based on lighting conditions
    z(1) = z(1) - 0.02; %may have to adjust this based on lighting conditions
    
    z(3) = 0.35; %fixed height to move to round
    
    %PID parameters for lateral movement without round
    Kp_x = 20;
    Ki_x = 20;
    Kd_x = 2.5;
    
    Kp_y = Kp_x;
    Ki_y = Ki_x;
    Kd_y = Kd_x;

    Kp_z = 50;
    Ki_z = 25;
    Kd_z = 1;
    
    
    tau = zeros(0,6); %clear previous effort values
    in_position = 0; %initialize variable to maintain the loop
    
    while in_position ~= 1
        tauLast = tau;

        fbk_move = getNextFeedback(group_move); %get motor information
        q=fbk_move.position'; %get motor position
        x=func_sym_arm_kinematics_3D(q); %end effector position

        COM = COMfinal(q); %find COM of robot based on each motor's position
        angleCOM = -angleFinal(q); %find angle of COM from base of robot
        radiusCOM = radiusFinal(q); %find distance of COM from base of robot
        tau2 = -(radiusCOM*cos(angleCOM)*sum(mass)*g)/2; %find effort needed by one of the two base lifting motors
        tau3 = -tau2; %use opposite value of effort needed by other base lifting motor

        pos4 = -pi/2; %lock out motor at the base of the first link
        tau5 = COM_masses(2)*cos(q(5)+q(3))*mass(2)*g; %find effort required by motor that controls second link

        pos6 = (-pi/2-(q(3)+q(5))); %find position required by motor at end of second link to have end-effector always pointed down

        dt = toc-t; %find timestep
        t = toc; %reinitiate previous time step
        
        z_t = x; %redefine end-effector position
        error_raw = z-z_t; %find raw error value in each dimension
        error  = error_raw/sqrt(error_raw'*error_raw); %normalize error in each dimension
        
        
        [error_raw error] %print error in each dimension
        
        error_P = error; %proportional error
        error_I = 1*(dt*error + sumError); %integral error
        error_D = 1/dt*(error - errorLast); %derivative error
        errorLast = error; %retain error for next timestep

        Kp = [Kp_x 0 0; 0 Kp_y 0; 0 0 Kp_z]; %matrix for each gain in each dimension
        Ki = [Ki_x 0 0; 0 Ki_y 0; 0 0 Ki_z]; %matrix for each gain in each dimension
        Kd = [Kd_x 0 0; 0 Kd_y 0; 0 0 Kd_z]; %matrix for each gain in each dimension
                
        vel_corrected = Kp*error_P + Ki*error_I + Kd*error_D; %find the corrected velocity
        
        J = func_sym_arm_jacobian_3D(q); %find Jacobian based on robot's current position
        vel_q = J'*(vel_corrected * 0.05); %find velocity required by each motor
        
        vel = [vel_q(1) vel_q(2) -vel_q(2) NaN vel_q(5) NaN]; %write velocity required by each motor into Hebi format
        [norm_vel factor] = normalized_velocity(vel,speed_max); %run velocity normalization function
        
        %Ends loop when robot is close enough to desired point        
        if (sqrt(error_raw(1)^2+error_raw(2)^2) < 0.03) && (error_raw(3) < 0.08)
            tau1 = 0;
            fprintf('holding\n')
            in_position = 1;
        end
        
        %Send commands previously calculated to Hebi
        cmd_move.position = [NaN NaN NaN pos4 NaN pos6];
        cmd_move.effort = tau;
        cmd_move.velocity = norm_vel;
        group_move.send(cmd_move);
        pause(0.01);
    end
    %% Go down
    fprintf('Go down\n')
    z = points(i,:)';
    z(3) = 0.075; % Height of round
    
    %PID gains for robot to go down
    Kp_x = 15;
    Ki_x = 8;
    Kd_x = 2.5;
    
    Kp_y = Kp_x;
    Ki_y = Ki_x;
    Kd_y = Kd_x;

    Kp_z = 10;
    Ki_z = 5;
    Kd_z = 2;

    tau = zeros(0,6);

    in_position = 0;
    tau1 = 0;
    
    %Code here is the same as previous while loop except for how it
    %determines when it is in position and the gains assigned
    while in_position ~= 1
        tauLast = tau;

        fbk_move = getNextFeedback(group_move);
        q=fbk_move.position';
        x=func_sym_arm_kinematics_3D(q);

        COM = COMfinal(q);
        angleCOM = -angleFinal(q);
        radiusCOM = radiusFinal(q);
        tau2 = -(radiusCOM*cos(angleCOM)*sum(mass)*g)/2;
        tau3 = -tau2;

        pos4 = -pi/2;
        tau5 = COM_masses(2)*cos(q(5)+q(3))*mass(2)*g;

        pos6 = (-pi/2-(q(3)+q(5)));

        dt = toc-t;
        t = toc;
        
        z_t = x;
        error_raw = z-z_t;
        error  = error_raw/sqrt(error_raw'*error_raw);
        
        [error_raw error]
        
        error_P = error;
        
        error_I = 1*(dt*error + sumError);
        error_D = 1/dt*(error - errorLast); 
        errorLast = error;

        Kp = [Kp_x 0 0; 0 Kp_y 0; 0 0 Kp_z];
        Ki = [Ki_x 0 0; 0 Ki_y 0; 0 0 Ki_z];
        Kd = [Kd_x 0 0; 0 Kd_y 0; 0 0 Kd_z];
        
        vel_corrected = Kp*error_P + Ki*error_I + Kd*error_D;
        
        J = func_sym_arm_jacobian_3D(q);
        vel_q = J'*(vel_corrected * 0.03);
        
        vel = [vel_q(1) vel_q(2) -vel_q(2) NaN vel_q(5) NaN];
        [norm_vel factor] = normalized_velocity(vel,speed_max);
        norm_vel = vel;
        
        %Breakout code when robot is close enough to desired point
        if abs(error_raw(3)) < 0.02
            tau1 = 0;
            fprintf('holding\n')
            in_position = 1;
        end
        
        cmd_move.position = [NaN NaN NaN pos4 NaN pos6];
        cmd_move.effort = tau;
        cmd_move.velocity = norm_vel;
        group_move.send(cmd_move);
        pause(0.01);
    end
    %% Grip round
    %Code to grip round
    fprintf('Grip round\n')
    in_position = 0;
    while in_position ~= 1
    time_now = toc;
        while toc-time_now < 1.2
            cmd_gripper.position = [0.4];
            group_gripper.send(cmd_gripper);
            toc - time_now;
        end
    in_position = 1;
    end
    %% Go up w/round
    fprintf('Go up w/round\n')
    z = points(i,:)';
    z(3) = 0.35; %height of robot to move with round
    
    %PID gains for going up with a round    
    Kp_x = 15;
    Ki_x = 8;
    Kd_x = 2.5;
    
    Kp_y = Kp_x;
    Ki_y = Ki_x;
    Kd_y = Kd_x;

    Kp_z = 18;
    Ki_z = 10;
    Kd_z = 1;

    
    tau = zeros(0,6);
    in_position = 0;
    tau1 = 0;
    sumError_z = 0;
    errorLast_z = 0;
    
    while in_position ~= 1
        tauLast = tau;

        fbk_move = getNextFeedback(group_move);
        q=fbk_move.position';
        x=func_sym_arm_kinematics_3D(q);

        COM = COMfinal_round(q);
        angleCOM = -angleFinal_round(q);
        radiusCOM = radiusFinal_round(q);
        tau2 = -(radiusCOM*cos(angleCOM)*sum(mass_round)*g)/2;
        tau3 = -tau2;

        pos4 = -pi/2;
        tau5 = COM_masses_round(2)*cos(q(5)+q(3))*mass_round(2)*g;

        pos6 = (-pi/2-(q(3)+q(5)));

        dt = toc-t;
        t = toc;
        
        z_t = x;
        error_raw = z-z_t;
        error  = error_raw/sqrt(error_raw'*error_raw);
        
        [error_raw error]
        
        error_P = error;
        
        error_I = 1*(dt*error + sumError);
        error_D = 1/dt*(error - errorLast); 
        errorLast = error;

        Kp = [Kp_x 0 0; 0 Kp_y 0; 0 0 Kp_z];
        Ki = [Ki_x 0 0; 0 Ki_y 0; 0 0 Ki_z];
        Kd = [Kd_x 0 0; 0 Kd_y 0; 0 0 Kd_z];
        
        vel_corrected = Kp*error_P + Ki*error_I + Kd*error_D;
        
        J = func_sym_arm_jacobian_3D(q);
        vel_q = J'*(vel_corrected.*[1 1 1]'); %[1 1 1]' matrix was used to "boost" velocity in the vertical direction on previous iterations, i.e. it was set to [1 1 1.2]' to give the velocity upwards more power (leaving this in because it may be useful)
        
        vel = [vel_q(1) vel_q(2) -vel_q(2) NaN vel_q(5) NaN];
        [norm_vel factor] = normalized_velocity(vel,speed_max);
        
        tau = [tau1 tau2 tau3 NaN tau5 NaN];
        
        if abs(error_raw(3)) < 0.05
            tau1 = 0;
            fprintf('holding\n')
            in_position = 1;
        end
        
        cmd_move.position = [NaN NaN NaN pos4 NaN pos6];
        cmd_move.effort = tau;
        cmd_move.velocity = norm_vel;
        group_move.send(cmd_move);
        
        %gripper holds entire time robot has round
        cmd_gripper.position = [0.4];
        group_gripper.send(cmd_gripper);
        pause(0.01);
    end
    %% Move round
    fprintf('Move round\n')
    z(1:2) = destination(i,:)';
      
    Kp_x = 2;
    Ki_x = 0.2;
    Kd_x = 1;
    
    Kp_y = Kp_x;
    Ki_y = Ki_x;
    Kd_y = Kd_x;

    Kp_z = 15;
    Ki_z = 8;
    Kd_z = 0.1;
    

    in_position = 0;
    tau1 = 0;
    
    sumError_z = 0;
    errorLast_z = 0;
    sumError_xy = 0;
    errorLast_xy = 0;
    
    
    while in_position ~= 1
        tauLast = tau;

        fbk_move = getNextFeedback(group_move);
        q=fbk_move.position';
        x=func_sym_arm_kinematics_3D(q);

        COM = COMfinal_round(q);
        angleCOM = -angleFinal_round(q);
        radiusCOM = radiusFinal_round(q);
        tau2 = -(radiusCOM*cos(angleCOM)*sum(mass_round)*g)/2;
        tau3 = -tau2;

        pos4 = -pi/2;
        tau5 = COM_masses_round(2)*cos(q(5)+q(3))*mass_round(2)*g;

        pos6 = (-pi/2-(q(3)+q(5)));

        dt = toc-t;
        t = toc;
        
        z_t = x;
        error_raw = z-z_t;
        error  = error_raw/sqrt(error_raw'*error_raw); 
        
        [error_raw error]
        
        error_P = error;
        
        error_I = 1*(dt*error + sumError);
        error_D = 1/dt*(error - errorLast); 
        errorLast = error;

        Kp = [Kp_x 0 0; 0 Kp_y 0; 0 0 Kp_z];
        Ki = [Ki_x 0 0; 0 Ki_y 0; 0 0 Ki_z];
        Kd = [Kd_x 0 0; 0 Kd_y 0; 0 0 Kd_z];

        vel_corrected = Kp*error_P + Ki*error_I + Kd*error_D;
        
        J = func_sym_arm_jacobian_3D(q);
        vel_q = J'*(vel_corrected.*[1 1 1]');
        
        vel = [vel_q(1) vel_q(2) -vel_q(2) NaN vel_q(5) NaN];
        [norm_vel factor] = normalized_velocity(vel,speed_max);

        tau = [tau1 tau2 tau3 NaN tau5 NaN];
        
        if (sqrt(error_raw(1)^2+error_raw(2)^2) < 0.03) && (error_raw(3) < 0.08)
            tau1 = 0;
            fprintf('holding\n')
            in_position = 1;
        end
        
        cmd_move.position = [NaN NaN NaN pos4 NaN pos6];
        cmd_move.effort = tau;
        cmd_move.velocity = norm_vel;
        group_move.send(cmd_move);
        
        [cmd_move.velocity; cmd_move.effort];
        
        cmd_gripper.position = [0.4];
        group_gripper.send(cmd_gripper);
        pause(0.01);
    end
    %% Lower round
    fprintf('Lower round\n')
    z(3) = 0.08;
    
    Kp_x = 25;
    Ki_x = 8;
    Kd_x = 1.0;
    
    Kp_y = Kp_x;
    Ki_y = Ki_x;
    Kd_y = Kd_x;

    Kp_z = 0.1;
    Ki_z = 0;
    Kd_z = 0;

    in_position = 0;
    tau1 = 0;
    
    sumError_z = 0;
    errorLast_z = 0;
    sumError_xy = 0;
    errorLast_xy = 0;
    
    while in_position ~= 1
        tauLast = tau;

        fbk_move = getNextFeedback(group_move);
        q=fbk_move.position';
        x=func_sym_arm_kinematics_3D(q);

        COM = COMfinal_round(q);
        angleCOM = -angleFinal_round(q);
        radiusCOM = radiusFinal_round(q);
        tau2 = -(radiusCOM*cos(angleCOM)*sum(mass_round)*g)/2;
        tau3 = -tau2;

        pos4 = -pi/2;
        tau5 = COM_masses_round(2)*cos(q(5)+q(3))*mass_round(2)*g;

        pos6 = (-pi/2-(q(3)+q(5)));

        dt = toc-t;
        t = toc;
        
        z_t = x;
        error_raw = z-z_t;
        error  = error_raw/sqrt(error_raw'*error_raw); 
        
        [error_raw error]
        
        error_P = error;
        
        error_I = 1*(dt*error + sumError);
        error_D = 1/dt*(error - errorLast); 
        errorLast = error;

        Kp = [Kp_x 0 0; 0 Kp_y 0; 0 0 Kp_z];
        Ki = [Ki_x 0 0; 0 Ki_y 0; 0 0 Ki_z];
        Kd = [Kd_x 0 0; 0 Kd_y 0; 0 0 Kd_z];
                
        vel_corrected = Kp*error_P + Ki*error_I + Kd*error_D;
        
        J = func_sym_arm_jacobian_3D(q);
        vel_q = J'*(vel_corrected * 0.05);
        vel_q = J'*(vel_corrected);
        
        vel = [vel_q(1) vel_q(2) -vel_q(2) NaN vel_q(5) NaN];
        [norm_vel factor] = normalized_velocity(vel,speed_max);
        
        tau = [tau1 tau2 tau3 NaN tau5 NaN];
        
        if abs(error_raw(3)) < 0.03
            tau1 = 0;
            fprintf('holding\n')
            in_position = 1;
        end
        
        cmd_move.position = [NaN NaN NaN pos4 NaN pos6];
        cmd_move.effort = tau;
        cmd_move.velocity = norm_vel;
        group_move.send(cmd_move);
        
        cmd_gripper.position = [0.4];
        group_gripper.send(cmd_gripper);
        pause(0.01);
    end
    %% Release round
    fprintf('Release round\n')
    in_position = 0;
    while in_position ~= 1
    time_now = toc;
        while toc-time_now < 0.5
            cmd_gripper.position = [1];
            group_gripper.send(cmd_gripper);
            toc - time_now;
        end
    in_position = 1;
    end
    %% Go Up
    fprintf('Go up\n')
    z(3) = 0.35;
    
    Kp_x = 15;
    Ki_x = 12;
    Kd_x = 2.5;
    
    Kp_y = Kp_x;
    Ki_y = Ki_x;
    Kd_y = Kd_x;

    Kp_z = 5;
    Ki_z = 2;
    Kd_z = 0.1;

    in_position = 0;
    tau1 = 0;
    
    sumError_z = 0;
    errorLast_z = 0;
    sumError_xy = 0;
    errorLast_xy = 0;
    
    
    %This while loop has torque compensation, not sure it is needed but it
    %can be taken out/experimented with
    while in_position ~= 1
        tauLast = tau;

        fbk_move = getNextFeedback(group_move);
        q=fbk_move.position';
        x=func_sym_arm_kinematics_3D(q);

        COM = COMfinal(q);
        angleCOM = -angleFinal(q);
        radiusCOM = radiusFinal(q);
        tau2 = -(radiusCOM*cos(angleCOM)*sum(mass)*g)/2;
        tau3 = -tau2;

        pos4 = -pi/2;
        tau5 = COM_masses(2)*cos(q(5)+q(3))*mass(2)*g;

        pos6 = (-pi/2-(q(3)+q(5)));

        dt = toc-t;
        t = toc;
        
        z_t = x;
        error_raw = z-z_t;
        error  = error_raw/sqrt(error_raw'*error_raw);
        
        [error_raw error]
        
        error_P = error;
        
        error_I = 1*(dt*error + sumError);
        error_D = 1/dt*(error - errorLast); 
        errorLast = error;

        Kp = [Kp_x 0 0; 0 Kp_y 0; 0 0 Kp_z];
        Ki = [Ki_x 0 0; 0 Ki_y 0; 0 0 Ki_z];
        Kd = [Kd_x 0 0; 0 Kd_y 0; 0 0 Kd_z];
                
        vel_corrected = Kp*error_P + Ki*error_I + Kd*error_D;
        
        J = func_sym_arm_jacobian_3D(q);
        vel_q = J'*(vel_corrected * 0.03);
        vel_q = J'*(vel_corrected);
        
        vel = [vel_q(1) vel_q(2) -vel_q(2) NaN vel_q(5) NaN];
        [norm_vel factor] = normalized_velocity(vel,speed_max);
        
        %Torque compensation for round
        %Z torque
        Kp_tau_z = 0.2;
        Ki_tau_z= 0.3;
        Kd_tau_z = 0.01;
        error_z = error(3);
        sumError_z = 1*(dt*error_z + sumError_z);
        derivative_z = dt*(error_z - errorLast_z); 
        errorLast_z = error_z; 
        
        P_z = error_z*Kp_tau_z;
        I_z = sumError_z*Ki_tau_z;
        D_z = derivative_z*Kd_tau_z;
    
        Z_force = P_z + I_z + D_z;
        %XY torque
        Kp_tau_xy = 5;
        Ki_tau_xy = 0.2;
        Kd_tau_xy = 3;
        error_xy = sqrt(error(1)^2+error(2)^2);
        sumError_xy = 1*(dt*error_xy + sumError_xy);
        derivative_xy = 1/dt*(error_xy - errorLast_xy); 
        errorLast_xy = error_xy; 
        
        P_xy = error_xy*Kp_tau_xy;
        I_xy = sumError_xy*Ki_tau_xy;
        D_xy = derivative_xy*Kd_tau_xy;

        xy_ang = atan2(error(2),error(1));
        X_force = (P_xy + I_xy + D_xy)*cos(xy_ang);
        Y_force = (P_xy + I_xy + D_xy)*sin(xy_ang);        
        
        tau_error = [X_force; Y_force; Z_force];

        tau_gain = J'*tau_error;
    
        tau = [NaN tau2 tau3 NaN tau5 NaN];

        tau2 = tau2+tau_gain(2);
        tau3 = -tau2;
        tau5 = tau5+tau_gain(5);       
        tau1 = tau_gain(1);
        
        tau = [tau1 tau2 tau3 NaN tau5 NaN];

        
        if (sqrt(error_raw(1)^2+error_raw(2)^2) < 0.08) && (error_raw(3) < 0.08)
            tau1 = 0;
            fprintf('holding\n')
            in_position = 1;
        end
        
        cmd_move.position = [NaN NaN NaN pos4 NaN pos6];
        cmd_move.effort = tau;
        cmd_move.velocity = norm_vel;
        group_move.send(cmd_move);
        
        cmd_gripper.position = [1];
        group_gripper.send(cmd_gripper);
        pause(0.01);
    end
end

%% Hold
%This code returns the robot close to the original position after all
%rounds have been moved.
fprintf('Go to start')
    z = [0.0944; 0.6699; 0.35];
    
    Kp_x = 15;
    Ki_x = 20;
    Kd_x = 2.5;
    
    Kp_y = Kp_x;
    Ki_y = Ki_x;
    Kd_y = Kd_x;

    Kp_z = 50;
    Ki_z = 30;
    Kd_z = 1;

    tau = zeros(0,6);

    in_position = 0;
    tau1 = 0;
    
    while in_position ~= 1
        tauLast = tau;

        fbk_move = getNextFeedback(group_move);
        q=fbk_move.position';
        x=func_sym_arm_kinematics_3D(q);

        COM = COMfinal(q);
        angleCOM = -angleFinal(q);
        radiusCOM = radiusFinal(q);
        tau2 = -(radiusCOM*cos(angleCOM)*sum(mass)*g)/2;
        tau3 = -tau2;

        pos4 = -pi/2;
        tau5 = COM_masses(2)*cos(q(5)+q(3))*mass(2)*g;

        pos6 = (-pi/2-(q(3)+q(5)));

        dt = toc-t;
        t = toc;
        
        z_t = x;
        error_raw = z-z_t;
        error  = error_raw/sqrt(error_raw'*error_raw); 
        
        [error_raw error]
        
        error_P = error;
        
        error_I = 1*(dt*error + sumError);
        error_D = 1/dt*(error - errorLast); 
        errorLast = error;

        Kp = [Kp_x 0 0; 0 Kp_y 0; 0 0 Kp_z];
        Ki = [Ki_x 0 0; 0 Ki_y 0; 0 0 Ki_z];
        Kd = [Kd_x 0 0; 0 Kd_y 0; 0 0 Kd_z];
        
        vel_corrected = Kp*error_P + Ki*error_I + Kd*error_D;
        
        J = func_sym_arm_jacobian_3D(q);
        vel_q = J'*(vel_corrected * 0.05);
        
        vel = [vel_q(1) vel_q(2) -vel_q(2) NaN vel_q(5) NaN];
        [norm_vel factor] = normalized_velocity(vel,speed_max);
        
        if (sqrt(error_raw(1)^2+error_raw(2)^2) < 0.07) && (error_raw(3) < 0.10)
            tau1 = 0;
            fprintf('holding\n')
            in_position = 1;
        end
        
        cmd_move.position = [NaN NaN NaN pos4 NaN pos6];
        cmd_move.effort = tau;
        cmd_move.velocity = norm_vel;
        group_move.send(cmd_move);
        pause(0.01);
    end
 
%locks robot in position after getting close to start/safety position
in_position = 0;
while in_position ~= 1
    pos4 = -pi/2;
    pos6 = (-pi/2-(q(3)+q(5)));
    new_q = [1.6963 -1.0191 1.0191 pos4 -1.0472 pos6];
    cmd_move.position = new_q;
    group_move.send(cmd_move);
    pause(0.01);
end