function radiusDemo = radiusDemo(in1)
%RADIUSDEMO
%    RADIUSDEMO = RADIUSDEMO(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    07-Jan-2022 10:18:57

q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q2);
t3 = cos(q3);
t4 = sin(q2);
t5 = sin(q3);
radiusDemo = sqrt((t2.*2.735671957671958e-1+t2.*t3.*2.334603174603175e-2-t4.*t5.*2.334603174603175e-2).^2+(t4.*2.735671957671958e-1+t2.*t5.*2.334603174603175e-2+t3.*t4.*2.334603174603175e-2).^2);
