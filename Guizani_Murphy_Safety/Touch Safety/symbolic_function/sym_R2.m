function R2 = sym_R2(th)
%SYM_R2
%    R2 = SYM_R2(TH)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    29-Aug-2021 17:10:45

t2 = cos(th);
t3 = sin(th);
R2 = reshape([t2,0.0,-t3,0.0,1.0,0.0,t3,0.0,t2],[3,3]);
