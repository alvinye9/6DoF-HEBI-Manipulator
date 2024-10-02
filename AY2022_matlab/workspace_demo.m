close all; clear all; clc; format compact; 
addpath('hebi','symbolic_functions')%','slprj',
%git change
%% Hebi Setup
joy = vrjoystick(1) ;

% move group setup
serials_move = {     'x-80663' %rotation base
                     'x-81208' %shoulder 1a
                     'x-81128' %shoulder 1b
                     'x-80978' %shoulder 2
                     'x-80760' %intermediate
                     'x-81034' %leveler
                };
group_move = HebiLookup.newGroupFromSerialNumbers(serials_move);
cmd_move = CommandStruct();
fbk_move = getNextFeedback(group_move);                                                                     

% gripper group setup
serials_gripper = {'x-01075'}; %gripper
group_gripper = HebiLookup.newGroupFromSerialNumbers(serials_gripper);
cmd_gripper = CommandStruct();
fbk_gripper = getNextFeedback(group_gripper);

fprintf('hebi setup\n')

%% position determination
q=fbk_move.position; x=func_sym_arm_kinematics_3D(q');

% Start Position
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

speed_max = 1.0; %max speed of robot

X = 0; %while loop variable, press x on xbox controller to move to workspace demo

round_engaged = 0; %initiate control of robot without round
first = 0; %variable to tell robot if it has round or not

%Joint space demo begins here
while ~ X

    %% Feedback
    fbk_move = getNextFeedback(group_move);
    q=fbk_move.position';
    v=fbk_move.velocity;
    x=func_sym_arm_kinematics_3D(q) ;
    
    [ buttons , axes ] = get_input( joy ) ;% get button X and desired speed
    X = buttons(3) ; %if x is pressed, breaks out of while loop and goes to workspace demo

    
    %Code to determine if round is gripped
    if axes(3) >= 0.3 && first == 0
            first = 1;
            round_engaged = 1;
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
        end
        
        if axes(3) <= 0.3 && first == 1
            first = 0
            round_engaged = 0;
            in_position = 0;
            while in_position ~= 1
                time_now = toc;
                while toc-time_now < 1.2
                    cmd_gripper.position = [1];
                    group_gripper.send(cmd_gripper);
                    toc - time_now;
                end
                in_position = 1;
            end
        end
    
    %if round is engaged, use COM for robot with round    
    if round_engaged == 1
        cmd_gripper.position = [0.4];
        COM = COMfinal_round(q);
        angleCOM = -angleFinal_round(q);
        radiusCOM = radiusFinal_round(q);
        tau5 = COM_masses_round(2)*cos(q(5)+q(3))*mass_round(2)*g;
        fprintf('round engaged\n')
    end
    %if round is not engaged, use COM for robot without round
    if round_engaged == 0
        cmd_gripper.position = [1];
        COM = COMfinal(q);
        angleCOM = -angleFinal(q);
        radiusCOM = radiusFinal(q);
        tau5 = COM_masses(2)*cos(q(5)+q(3))*mass(2)*g;
        fprintf('no round\n')
    end
    
    %%Calculations and sending speeds to Hebi
    tau2 = -(radiusCOM*cos(angleCOM)*sum(mass)*g)/2*1.05;
    tau3 = -tau2;
    cos((q(3)+ q(5)));
    pos4 = -pi/2;
    tau = [NaN tau2 tau3 NaN tau5 NaN];
    pos6 = (-pi/2-(q(3)+q(5)));
    velocity_final = [v(1) v(2) v(3) v(5)];
        
    group_gripper.send(cmd_gripper);
    
    joint_vel1 = axes(1)*speed_max;
    joint_vel2 = 2.5*axes(2)*speed_max;
    joint_vel3 = axes(5)*speed_max;
    
    cmd_move.velocity = [joint_vel1 joint_vel2 -joint_vel2 NaN -joint_vel3 NaN];
    cmd_move.position = [NaN NaN NaN pos4 NaN pos6];
    cmd_move.effort = [NaN tau2 tau3 NaN tau5 NaN];
    group_gripper.send(cmd_gripper);
    group_move.send(cmd_move);
    pause(0.01);

end

%initialization of workspace demo
speed_max = 1; %can change speed for workspace demo if desired

b = 0; %b button on xbox breaks out of loop and kills robot 

%workspace translation of end effector from robot joints
q=fbk_move.position';
z = func_sym_arm_kinematics_3D(q);

%reinitialize variables that tell robot if it has round
round_engaged = 0;
first = 0;

%workspace demo starts here
while ~b
        
        tau = zeros(0,6);
        tau1 = 0;

        [ buttons , axes ] = get_input( joy ) ;% get button B and desired speed

        b = buttons(2) ;% quit with B         
        
        %tells robot if it has round or not, same as jointspace
        if axes(3) >= 0.3 && first == 0
            first = 1;
            round_engaged = 1;
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
        end
        
        if axes(3) <= 0.3 && first == 1
            first = 0
            round_engaged = 0;
            in_position = 0;
            while in_position ~= 1
                time_now = toc;
                while toc-time_now < 1.2
                    cmd_gripper.position = [1];
                    group_gripper.send(cmd_gripper);
                    toc - time_now;
                end
                in_position = 1;
            end
        end

        z = z + 0.01*[axes(2)+0.0084 axes(1)-0.129 -axes(5)-0.0261]'; %give desired positions, numbers added to each axes are to correct for stick in controller
        
        tauLast = tau;
        
        fbk_move = getNextFeedback(group_move);
        q=fbk_move.position';
        x=func_sym_arm_kinematics_3D(q);

        %if no round is present, use this code
        if round_engaged == 0
            Kp_x = 15;
            Ki_x = 20;
            Kd_x = 2.5;

            Kp_y = Kp_x;
            Ki_y = Ki_x;
            Kd_y = Kd_x;

            Kp_z = 50;
            Ki_z = 30;
            Kd_z = 1;     

            COM = COMfinal(q);
            angleCOM = -angleFinal(q);
            radiusCOM = radiusFinal(q);
            tau2 = -(radiusCOM*cos(angleCOM)*sum(mass)*g)/2;
            tau3 = -tau2;

            pos4 = -pi/2;
            tau5 = COM_masses(2)*cos(q(5)+q(3))*mass(2)*g;

            pos6 = (-pi/2-(q(3)+q(5)));
                
        J = func_sym_arm_jacobian_3D(q);
        
        vel_corrected = [axes(2)+0.0084 axes(1)-0.129 -axes(5)-0.0261]';
        vel_q = J'*(vel_corrected * 1);

        
        vel = [vel_q(1) vel_q(2) -vel_q(2) NaN vel_q(5) NaN];
        
        norm_vel = vel;
        
        cmd_move.position = [NaN NaN NaN pos4 NaN pos6];
        cmd_move.effort = tau;
        cmd_move.velocity = norm_vel;
        group_move.send(cmd_move);
        cmd_gripper.position = [1];
        group_gripper.send(cmd_gripper);
        pause(0.01);
        end
        
        %if round is detected, use this code
        if round_engaged == 1
            Kp_x = 15;
            Ki_x = 20;
            Kd_x = 2.5;

            Kp_y = Kp_x;
            Ki_y = Ki_x;
            Kd_y = Kd_x;

            Kp_z = 50;
            Ki_z = 30;
            Kd_z = 1;     

            COM = COMfinal_round(q);
            angleCOM = -angleFinal_round(q);
            radiusCOM = radiusFinal_round(q);
            tau2 = -(radiusCOM*cos(angleCOM)*sum(mass_round)*g)/2;
            tau3 = -tau2;

            pos4 = -pi/2;
            tau5 = COM_masses_round(2)*cos(q(5)+q(3))*mass_round(2)*g;

            pos6 = (-pi/2-(q(3)+q(5)));
                
            J = func_sym_arm_jacobian_3D(q);

            vel_corrected = [axes(2)+0.0084 axes(1)-0.129 -axes(5)-0.0261]';
            vel_q = J'*(vel_corrected.* [1 1 1.4]');

            vel = [vel_q(1) vel_q(2) -vel_q(2) NaN vel_q(5) NaN];
            
            norm_vel = vel;

            cmd_move.position = [NaN NaN NaN pos4 NaN pos6];
            cmd_move.effort = tau;
            cmd_move.velocity = norm_vel;
            group_move.send(cmd_move);
            cmd_gripper.position = [0.4];
            group_gripper.send(cmd_gripper);
            pause(0.01);
        end
end

%% Functions

function [ buttons , axes ] = get_input( joy )

% Buttons from Xbox Controller

buttons = button(joy,[1:10]) ;

% Axes from Xbox Controller

axes = axis( joy,[1:5] ) ;

end
   