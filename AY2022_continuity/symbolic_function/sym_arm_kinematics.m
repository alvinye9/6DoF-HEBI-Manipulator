function d = sym_arm_kinematics(in1)
%SYM_ARM_KINEMATICS
%    D = SYM_ARM_KINEMATICS(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    27-Jan-2022 17:27:24

q1 = in1(1,:);
q2 = in1(2,:);
t2 = cos(q1);
t3 = cos(q2);
t4 = sin(q1);
d = reshape([0.0,0.0,9.0./1.0e+2,t4.*(1.1e+1./1.0e+2)+t2.*t3.*(3.1e+1./1.0e+2),t2.*(-1.1e+1./1.0e+2)+t3.*t4.*(3.1e+1./1.0e+2),sin(q2).*(3.1e+1./1.0e+2)+9.0./1.0e+2],[3,2]);
