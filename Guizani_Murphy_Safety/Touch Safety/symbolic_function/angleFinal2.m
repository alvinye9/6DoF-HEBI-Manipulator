function angleFinal2 = angleFinal2(in1)
%ANGLEFINAL2
%    ANGLEFINAL2 = ANGLEFINAL2(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    14-Apr-2022 10:31:55

q5 = in1(5,:);
angleFinal2 = atan2(-sin(q5),cos(q5));
