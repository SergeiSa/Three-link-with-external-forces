function J1 = g_InverseKinematics_J1(in1)
%G_INVERSEKINEMATICS_J1
%    J1 = G_INVERSEKINEMATICS_J1(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    24-Apr-2018 22:38:16

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = q1+q2+q3;
t3 = cos(t2);
t4 = t3.*(1.0./1.0e1);
t5 = q1+q2;
t6 = cos(t5);
t7 = t6.*(3.0./1.0e1);
t8 = sin(t2);
t9 = sin(t5);
J1 = reshape([t4+t7+cos(q1).*(1.0./2.0),t8.*(-1.0./1.0e1)-t9.*(3.0./1.0e1)-sin(q1).*(1.0./2.0),1.0,t4+t7,t8.*(-1.0./1.0e1)-t9.*(3.0./1.0e1),1.0,t4,t8.*(-1.0./1.0e1),1.0],[3,3]);