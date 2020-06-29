close all; clear; %clear classes;
clc; 

%Create user interfase object for SRD
SRD = SRDuserinterface();

%Generate dynamics eq.
ToLinearize = true; 
ToSimplify = true; 
SRD.ToOptimizeFunctions = true;
SRD.UseParallelizedSimplification = false;

% SymbolicEngine = SRD.GetSymbolicEngine;
% SymbolicEngine.LinearizationType = 'Naive';
% SRD.SaveSymbolicEngine(SymbolicEngine);

LinkArray = SRD.GetLinkArray;
SymbolicEngine = SRDSymbolicEngine(LinkArray);
SymbolicEngine.gravitational_constant = [0;0;0];
SRD.SaveSymbolicEngine(SymbolicEngine);
SRD.RecreateSymbolicEngine = false;

timerVal = tic;
SRD.DeriveEquationsForSimulation(ToLinearize, ToSimplify);
toc(timerVal);

SymbolicEngine = SRD.GetSymbolicEngine();
ExternalForcesEngine = SRDAddExternalForces(SymbolicEngine);

f = sym('f', [2, 1]);
L = SymbolicEngine.RetreaveLinkInLinkArray("Torso"); %link where the force is applied
r = L.AbsoluteFollower; %point where the force is applied
J = jacobian(r, SymbolicEngine.q);
Force = J'*[f(1); 0; f(2)];

ExternalForcesEngine.AddForce(Force, f);

SRD.SaveExternalForcesEngine(ExternalForcesEngine);


rC = SymbolicEngine.GetCoM;
matlabFunction(rC, 'File', 'g_rC', 'Vars', {SymbolicEngine.q});





