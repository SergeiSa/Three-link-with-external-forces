function out1 = g_control_ForcesForComputedTorqueController(in1,in2)
%G_CONTROL_FORCESFORCOMPUTEDTORQUECONTROLLER
%    OUT1 = G_CONTROL_FORCESFORCOMPUTEDTORQUECONTROLLER(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    24-Aug-2020 21:45:37

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
v1 = in2(1,:);
v2 = in2(2,:);
v3 = in2(3,:);
t2 = -q2;
t3 = -q3;
t4 = -v2;
t5 = -v3;
t6 = q1+t2;
t7 = q1+t3;
t8 = q2+t3;
t9 = t4+v1;
t10 = t5+v1;
t11 = t5+v2;
t12 = sin(t6);
t13 = sin(t7);
t14 = sin(t8);
out1 = [v1-t9.*t12.*v2.*(3.0./4.0)-(t10.*t13.*v3)./4.0;v2-t9.*t12.*v1.*(3.0./4.0)-(t11.*t14.*v3)./4.0;v3-(t10.*t13.*v1)./4.0-(t11.*t14.*v2)./4.0];
