clear all; close all; clc; format compact;
syms('q',[5,1]);

%constants
l1 = 0.48;
l2 = 0.33;
% l3 = 0.315;
h1 = 0.09;
h2 = 0.11;
h3 = 0;

DH = [q1 , 0  , 0  , -pi/2 ;...
      q2 , 0  , l1 , q4+pi/2     ;...
      0  , -h1  , 0  , 0    ;...
      -q5 , 0 , l2 , 0;
      0 , -h2 , 0, 0 ]; 
  
numArrays = size(DH);
H = cell(numArrays(1),1);
for n = 1:numArrays
    H{n} = from_DH_to_H(DH(n,:));
end  

Z = cell(numArrays(1),1);
Z{1} = H{1};

for n = 2:length(H)
   Z{n,1} = Z{n-1,1}*H{n,1};
end

% H_01 = from_DH_to_T(DH(1,:));
% H_12 = from_DH_to_T(DH(2,:));
% H_02 = H_01*H_12 ;
% H_23 = from_DH_to_T(DH(3,:));
% H_03 = H_01*H_12*H_23;
% H_03 = simplify(H_03);
% H_34 = from_DH_to_T(DH(4,:));
% H_04 = H_01*H_12*H_23*H_34;
% d_04 = H_04(1:3,end);

d_int = Z{length(H),1};
d = d_int(1:3,end);

pos = cell(3,length(H)+1);
pos{1,1} = 0;
pos{2,1} = 0;
pos{3,1} = 0;

for n = 1:length(H)
   z_int = Z{n,:};
   z = z_int(1:3,end);
   pos{1,n+1} = z(1);
   pos{2,n+1} = z(2);
   pos{3,n+1} = z(3);
end

% position = [ zeros(3,1) , H_01(1:3,end) , H_02(1:3,end), H_03(1:3,end),  H_04(1:3,end)];
% 
% if pos == position
%     fprintf('true')
% end

matlabFunction( d , pos ,'File','symbolic_function/func_sym_arm_kinematics_3D','Vars',{q});
J = simplify(jacobian(d,q));
matlabFunction(J,'File','symbolic_function/func_sym_arm_jacobian_3D','Vars',{q});