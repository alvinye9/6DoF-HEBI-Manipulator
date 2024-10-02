clear all; close all; clc; format compact;
syms('q',[5,1]);

%constants
l1 = 0.48;
l2 = 0.33;
% l3 = 0.315;
h1 = 0.09;
h2 = 0.11;
h3 = 0;

%Array
A = [1.37, 0.848, 0.454, 0 , 0;...    %L1 masses
      0.095, 0.29, 0.44, 0 , 0;...   %L1 distances
      0.095, 0.069, 0.454, 0.129, 0.552;...   %L2 masses
      0.315, 0.015, 0.311, 0.163, 0.33];  %L2 distances

[a,b] = size(A);

COM = zeros(a/2,1);
mass = zeros(a/2,1);
for n = 1:length(A)/2
    %break up array into each arm
    p = n*2;
    B = A(p-1:p,:);
    %do COM calculation for each arm
    num = 0;
    den = 0;
    for i = 1:length(B)
        num = num + B(1,i)*B(2,i);
    end
    den = sum(B(1,:),2);
    mass(n,1) = den;
    COM(n,1) = num/den;
end

DH_COM1 = [q1 , 0  , 0  , -pi/2 ;...
      q2 , 0  , COM(1) , q4+pi/2 ];
     

% H_01COM1 = from_DH_to_T(DH_COM1(1,:));
H_12COM1 = from_DH_to_H(DH_COM1(2,:));
d_12COM1 = H_12COM1(1:3,end);

% H_02COM1 = H_01COM1*H_12COM1;
% d_01COM1 = H_02COM1(1:3,end);  


DH_COM2 = [q1 , 0  , 0  , -pi/2 ;...
      q2 , 0  , l1 , q4+pi/2     ;...
      0  , -h1  , 0  , 0    ;...
      -q5 , 0 , COM(2) , 0]; 
 
% H_01COM2 = from_DH_to_T(DH_COM2(1,:));
H_12COM2 = from_DH_to_H(DH_COM2(2,:));
H_23COM2 = from_DH_to_H(DH_COM2(3,:));
H_34COM2 = from_DH_to_H(DH_COM2(4,:));
% H_04COM2 = H_01COM2*H_12COM2*H_23COM2*H_34COM2;
% d_04COM2 = H_04COM2(1:3,end);
H_14COM2 = H_12COM2*H_23COM2*H_34COM2;
d_14COM2 = H_14COM2(1:3,end);
H_24COM2 = H_23COM2*H_34COM2;
d_24COM2 = H_24COM2(1:3,end);


COMfinal = (d_12COM1*mass(1)+d_14COM2*mass(2))/(sum(mass));
matlabFunction(COMfinal, 'File','symbolic_function/COMfinal','Vars',{q});

length = COMfinal(1);
matlabFunction(length, 'File','symbolic_function/radiusFinal','Vars',{q});

radiusFinal = sqrt(COMfinal(1)^2+COMfinal(2)^2);
matlabFunction(radiusFinal, 'File','symbolic_function/radiusFinal','Vars',{q});

angleFinal = atan2(COMfinal(2),COMfinal(1));
matlabFunction(angleFinal, 'File','symbolic_function/angleFinal','Vars',{q});

COMfinal2 = d_24COM2;
matlabFunction(COMfinal2, 'File','symbolic_function/COMfinal2','Vars',{q});

radiusFinal2 = sqrt(COMfinal2(1)^2+COMfinal2(2)^2);
matlabFunction(radiusFinal2, 'File','symbolic_function/radiusFinal2','Vars',{q});

angleFinal2 = atan2(COMfinal2(2),COMfinal2(1));
matlabFunction(angleFinal2, 'File','symbolic_function/angleFinal2','Vars',{q});
