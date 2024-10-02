clear; clc; close all; format compact; 
addpath('hebi','symbolic_function','functions')

%% Hebi Setup

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
 joy = vrjoystick(1);
 fbk_move = getNextFeedback(group_move);

%% position determination
 q=fbk_move.position; x=func_sym_arm_kinematics_3D(q');

%% Initial Setup

COM_masses = [0.2155; 0.2889];
mass = [2.6720;1.2990];
g = 9.81;
speed_max = 1.0;
killcode = 0;
int_detect = 0;
b = 0;
A = 0;
y = 0;
tic;

%% Skander ROS Setup

matlab_talker = rospublisher('/chatter','std_msgs/Float64MultiArray');
matlab_message = rosmessage( matlab_talker );

matlab_listens = rossubscriber('/skander_bot_5000');
pause(2)
listener_msg = matlab_listens;

%% Vision and Touch Main Code
while ~y
while ~b
    
   fprintf('Safety and Touch Demo\n')
%% Feedback

    fbk_move = getNextFeedback(group_move);
    q=fbk_move.position';
    v=fbk_move.velocity;
    x=func_sym_arm_kinematics_3D(q);%print end effector position
    d = sym_arm_kinematics(q);
    t_old = toc;
    
%% Control

    COM = COMfinal(q);
    COM2 = COMfinal2(q);
    angleCOM = -angleFinal(q);
    angleCOM2 = -(angleFinal2(q)-q(3));
    radiusCOM = radiusFinal(q);
    radiusCOM2 = radiusFinal2(q);
    tau2 = -(radiusCOM*cos(angleCOM)*sum(mass)*g)/2*1.05;
    tau3 = -tau2;
    pos4 = -pi/2;
    tau5 = 1.07*COM_masses(2)*cos(q(5)+q(3))*mass(2)*g;
    pos6 = (-pi/2-(q(3)+q(5)));
    
%% ROS to Skander
    
    [time,issimtime] = rostime("now") ;
    ROS_time = time.Sec + time.Nsec/1e9 ;
    matlab_array = [d(1,1),d(2,1),d(3,1),1.0,d(1,2),d(2,2),d(3,2),1.0,d(1,3),d(2,3),d(3,3),1.0,d(1,4),d(2,4),d(3,4),1.0,d(1,5),d(2,5),d(3,5),1.0,d(1,6),d(2,6),d(3,6),1.0,d(1,7),d(2,7),d(3,7),1.0,d(1,8),d(2,8),d(3,8),1.0,d(1,9),d(2,9),d(3,9),1.0,d(1,10),d(2,10),d(3,10),1.0,d(1,11),d(2,11),d(3,11),1.0,d(1,12),d(2,12),d(3,12),1.0,d(1,13),d(2,13),d(3,13),1.0,d(1,14),d(2,14),d(3,14),1.0,d(1,15),d(2,15),d(3,15),1.0,d(1,16),d(2,16),d(3,16),1.0,d(1,17),d(2,17),d(3,17),1.0,d(1,18),d(2,18),d(3,18),1.0,d(1,19),d(2,19),d(3,19),1.0,d(1,20),d(2,20),d(3,20),1.0,d(1,21),d(2,21),d(3,21),1.0,d(1,22),d(2,22),d(3,22),1.0,d(1,23),d(2,23),d(3,23),1.0,d(1,24),d(2,24),d(3,24),1.0,d(1,25),d(2,25),d(3,25),1.0,d(1,26),d(2,26),d(3,26),1.0,d(1,27),d(2,27),d(3,27),1.0,d(1,28),d(2,28),d(3,28),1.0,d(1,29),d(2,29),d(3,29),1.0,d(1,30),d(2,30),d(3,30),1.0,d(1,31),d(2,31),d(3,31),1.0,d(1,32),d(2,32),d(3,32),1.0]; 
    matlab_message.Data = matlab_array ;
    send( matlab_talker , matlab_message );
    
%% Skander to ROS

    msg = matlab_listens.LatestMessage.Data;
    
%% Weight Code
    
    t = fbk_move.effort;
    
    rotation_base = t(1);
    
    mass_end1_1 = (((2*-t(2))/(radiusCOM*cos(angleCOM)*g)))-(1.05*sum(mass));
    
    mass_end1_2 = (((2*t(3))/(radiusCOM*cos(angleCOM)*g)))-(1.05*sum(mass));
    
    mass_end2_1 = (((t(5))/(radiusCOM2*cos(angleCOM2)*g)))-(1.05*mass(2));
    
%% Vision/Touch Safety Code
    
    if msg == 1.0
        killcode = 1;
        fprintf('Vision Safety Enacted\n')
    end
    
    if (abs(mass_end1_1) > 1.25 || abs(mass_end2_1) > 0.50 || abs(rotation_base) > 1.50) && int_detect == 0 
        toc;
    end
    
    if (abs(mass_end1_1) > 1.25 || abs(mass_end2_1) > 0.50 || abs(rotation_base) > 1.50) && toc >= 0.40 
        killcode = 1;
        fprintf('Touch Safety Enacted\n')
    end
    
    if (abs(mass_end1_1) > 1.25 || abs(mass_end2_1) > 0.50 || abs(rotation_base) > 1.50) < 0.60  
        int_detect = 0;
        tic;
    end
    
%% Movement/Killcode Control

    [buttons , axes] = get_input( joy );
    A = buttons(1);
    b = buttons(2);

    [buttons , axes] = get_input( joy );
    
    joint_vel1 = axes(1)*speed_max;
    joint_vel2 = axes(2)*speed_max;
    joint_vel3 = axes(5)*speed_max;
   
    cmd_move.velocity = [joint_vel1 joint_vel2 -joint_vel2 NaN -joint_vel3 NaN];
    cmd_move.position = [NaN NaN NaN pos4 NaN pos6];
    cmd_move.effort = [NaN tau2 tau3 NaN tau5 NaN];
    
    while killcode && A == 0
        q=fbk_move.position;
        cmd_move.position = [q];
        cmd_move.velocity = [0 0 0 0 0 0];
        [buttons, axes] = get_input(joy);
        A = buttons(1);
        group_move.send(cmd_move);
        pause(0.01);
    end
    
    group_move.send(cmd_move);
    pause(0.01);
    killcode = 0;
end

%% Touch Only Main Code

while ~A
    fprintf('Touch Demo\n')
    
    %% Feedback

    fbk_move = getNextFeedback(group_move);
    q=fbk_move.position';
    v=fbk_move.velocity;
    x=func_sym_arm_kinematics_3D(q) ;%print end effector position
    d = sym_arm_kinematics(q);
    t_old = toc;
    
%% Control

    COM = COMfinal(q);
    COM2 = COMfinal2(q);
    angleCOM = -angleFinal(q);
    angleCOM2 = -(angleFinal2(q)-q(3));
    radiusCOM = radiusFinal(q);
    radiusCOM2 = radiusFinal2(q);
    tau2 = -(radiusCOM*cos(angleCOM)*sum(mass)*g)/2*1.05;
    tau3 = -tau2;
    pos4 = -pi/2;
    tau5 = 1.07*COM_masses(2)*cos(q(5)+q(3))*mass(2)*g;
    pos6 = (-pi/2-(q(3)+q(5)));
    
%% ROS to Skander
    
    [time,issimtime] = rostime("now") ;
    ROS_time = time.Sec + time.Nsec/1e9 ;
    matlab_array = [d(1,1),d(2,1),d(3,1),1.0,d(1,2),d(2,2),d(3,2),1.0,d(1,3),d(2,3),d(3,3),1.0,d(1,4),d(2,4),d(3,4),1.0,d(1,5),d(2,5),d(3,5),1.0,d(1,6),d(2,6),d(3,6),1.0,d(1,7),d(2,7),d(3,7),1.0,d(1,8),d(2,8),d(3,8),1.0,d(1,9),d(2,9),d(3,9),1.0,d(1,10),d(2,10),d(3,10),1.0,d(1,11),d(2,11),d(3,11),1.0,d(1,12),d(2,12),d(3,12),1.0,d(1,13),d(2,13),d(3,13),1.0,d(1,14),d(2,14),d(3,14),1.0,d(1,15),d(2,15),d(3,15),1.0,d(1,16),d(2,16),d(3,16),1.0,d(1,17),d(2,17),d(3,17),1.0,d(1,18),d(2,18),d(3,18),1.0,d(1,19),d(2,19),d(3,19),1.0,d(1,20),d(2,20),d(3,20),1.0,d(1,21),d(2,21),d(3,21),1.0,d(1,22),d(2,22),d(3,22),1.0,d(1,23),d(2,23),d(3,23),1.0,d(1,24),d(2,24),d(3,24),1.0,d(1,25),d(2,25),d(3,25),1.0,d(1,26),d(2,26),d(3,26),1.0,d(1,27),d(2,27),d(3,27),1.0,d(1,28),d(2,28),d(3,28),1.0,d(1,29),d(2,29),d(3,29),1.0,d(1,30),d(2,30),d(3,30),1.0,d(1,31),d(2,31),d(3,31),1.0,d(1,32),d(2,32),d(3,32),1.0];
    matlab_message.Data = matlab_array ;
    send( matlab_talker , matlab_message );
    
%% Weight Code
    
    t = fbk_move.effort;
    
    rotation_base = t(1);
    
    mass_end1_1 = (((2*-t(2))/(radiusCOM*cos(angleCOM)*g)))-(1.05*sum(mass));
    
    mass_end1_2 = (((2*t(3))/(radiusCOM*cos(angleCOM)*g)))-(1.05*sum(mass));
    
    mass_end2_1 = (((t(5))/(radiusCOM2*cos(angleCOM2)*g)))-(1.05*mass(2));
    
%% Touch Safety Code
    
    if (abs(mass_end1_1) > 1.25 || abs(mass_end2_1) > 0.50 || abs(rotation_base) > 1.50) && int_detect == 0 
        toc;
    end
    
    if (abs(mass_end1_1) > 1.25 || abs(mass_end2_1) > 0.50 || abs(rotation_base) > 1.50) && toc >= 0.40 
        killcode = 1;
        fprintf('Touch Safety Enacted\n')
    end
    
    if (abs(mass_end1_1) > 1.25 || abs(mass_end2_1) > 0.50 || abs(rotation_base) > 1.50) < 0.60  
        int_detect = 0;
        tic;
    end
    
%% Movement/Killcode Control

    [buttons , axes] = get_input( joy );
    A = buttons(1);
    b = buttons(2);

    [buttons , axes] = get_input( joy );
    
    joint_vel1 = axes(1)*speed_max;
    joint_vel2 = axes(2)*speed_max;
    joint_vel3 = axes(5)*speed_max;
   
    cmd_move.velocity = [joint_vel1 joint_vel2 -joint_vel2 NaN -joint_vel3 NaN];
    cmd_move.position = [NaN NaN NaN pos4 NaN pos6];
    cmd_move.effort = [NaN tau2 tau3 NaN tau5 NaN];
    
    while killcode && A == 0
        q=fbk_move.position;
        cmd_move.position = [q];
        cmd_move.velocity = [0 0 0 0 0 0];
        [buttons, axes] = get_input(joy);
        A = buttons(1);
        group_move.send(cmd_move);
        pause(0.01);
    end
    
    group_move.send(cmd_move);
    pause(0.01);
    killcode = 0;
end 
end
%% Functions

function [ buttons , axes ] = get_input( joy )   
    % Buttons from Xbox Controller
    buttons = button(joy,[1:10]) ;    
    % Axes from Xbox Controller    
    axes   = axis(  joy,[1:5] ) ; 
end





