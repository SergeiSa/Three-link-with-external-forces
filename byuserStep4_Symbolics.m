close all; clc;  clear; %clear classes;

%Create user interfase object for SRD
SRD = SRDuserinterface();

LinkArray = SRD.GetLinkArray;
SymbolicEngine = SRDSymbolicEngine(LinkArray);
SymbolicEngine.gravitational_constant = [0;0;0];
SRD.SaveSymbolicEngine(SymbolicEngine);

timerVal = tic;
SRD.DeriveEquationsForSimulation('UseCasadi', false, 'ToLinearize', true, 'ToSimplify', true, ...
    'ToRecreateSymbolicEngine', false, 'dissipation_coefficients', [], ...
    'NumberOfWorkers', 8, 'ToUseParallelizedSimplification', false, 'ToOptimizeFunctions', true);
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





