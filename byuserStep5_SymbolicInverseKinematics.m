close all; clear; %clear classes;
clc; 

%Create user interfase object for SRD
SRD = SRDuserinterface;
SymbolicEngine = SRD.GetSymbolicEngine();

%%%%%%%%%%%%
%construct inverse kinematics task
rK = SymbolicEngine.GeometryArray{3}.Link.AbsoluteFollower;

abs_phs3 = SymbolicEngine.q(3);
  
Task = [rK(1); rK(3); abs_phs3]; 
%%%%%%%%%

%Call function for creating SRD inverse kinematics engine
SRD.SetupSymbolicInverseKinematics(Task);