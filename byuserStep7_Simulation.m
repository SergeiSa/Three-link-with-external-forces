function byuserStep7_Simulation()
close all;

%Create user interface object for SRD
SRD = SRDuserinterface;

ExternalForcesEngine = SRD.GetExternalForcesEngine();

%Load SimulationEngine and set up the simulation parameters
SimulationEngine = SRD.GetSimulationEngine();

a = 10;
w = 10;
%     function f = GetExternalForces()
%         SensorData = SimulationEngine.SensorHandler.ReadCurrentData;
%         t = SensorData.t;
%         
%         f = [a*sin(w*t); 
%              a*sin(w*t + pi/2)];
%     end
    function f = GetExternalForces()
        SensorData = SimulationEngine.SensorHandler.ReadCurrentData;
        t = SensorData.t;
        
        if (t > 0.3) && (t < 0.7)
            f = [100; 100]*0;
        else
            f =[0; 0];
        end
    end

%Can use 'Euler', 'Taylor', 'Runge', 'Implicit Euler', 'DAE Taylor', 'DAE Runge';
SimulationEngine.CustomSolverType = 'User-provided';
    function OutputStructure = User_provided_solver()
        
        f = GetExternalForces();
        
        ExternalForcesEngine.UpdateModel(f, SimulationEngine.ModelHandler);
        
        OutputStructure = SimulationEngine.Solver_TaylorUpdate();
    end

SimulationEngine.User_provided_solver = @User_provided_solver;


SimulationEngine.IC.q = SimulationEngine.IC.q + rand(3, 1)*0.5;
% SimulationEngine.IC.v = SimulationEngine.IC.v + rand(3, 1)*0.5;
SimulationEngine.IC.v = zeros(3, 1);


%Load InverseKinematicsEngine
InverseKinematicsEngine = SRD.GetInverseKinematicsEngine();

SimulationEngine.Time = InverseKinematicsEngine.TimeEnd - 0.0;
% SimulationEngine.Time = 5;

ControlInput = @InverseKinematicsEngine.EvaluatePolynomialApproximation;
% ControlInput = SimulationEngine.GetPlugInput("Constant_ControlInput", 'value_q', [1.2; -1; 1.2]);
% ControlInput = SimulationEngine.GetPlugInput("Constant_IC_ControlInput");


%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%
% ControllerWrapper = SRDControllerWrapper();
% 
% Kp = eye(SimulationEngine.dof) * 1000;
% Kd = eye(SimulationEngine.dof) * 200;

% Kp = [1000 100 100;
%       100 1000 100;
%       100 100 1000];

%     function PDcontroller(~, ~)
%         
%         SensorData = SimulationEngine.SensorHandler.ReadCurrentData;
%         
%         e = SensorData.desired_q - SensorData.q;
%         de = SensorData.desired_v - SensorData.v;
%         
%         ControllerWrapper.u = Kp*e + Kd*de;
%     end
% 
% ControllerWrapper.Controller = @PDcontroller;
% Controller = ControllerWrapper;

%%%%%%%%%%%%%%%%%
%PD controller example
Controller = SimulationEngine.GetPDcontroller('Computed torque PD', 'Kp', eye(SimulationEngine.dof)*500, ...
                                                                    'Kd', eye(SimulationEngine.dof)*100);
% Can use .GetPDcontroller with 'PD', 'Varying gains PD

%%%%%%%%%%%%%%%%%
% Controller = SimulationEngine.GetLQRcontroller('LQR', 'unified_Q', 10000, 'unified_R', 1, ...
%     'ILQR_TimeStep', 0.1);

%%%%%%%%%%%%%%%%%
% Controller = SimulationEngine.GetMPcontroller('MP', 'unified_Q', 10000, 'unified_R', 1, ...
%     'NumberOfPredictionSteps', 10, 'MP_PredictionTimeStep', 0.005);

%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%


%     function Tester(~, ~)      
%         SensorData = SimulationEngine.SensorHandler.ReadCurrentData;
%         rC = g_rC(SensorData.q);       
%     end


%Simulate
tic
Res = SimulationEngine.Simulation(ControlInput, Controller);
%Can use .Simulation() and .SimulationStateSpace()
toc

Count = size(Res.SimulationOutput.Position, 1);
Res.rC = zeros(Count, 2);
for i = 1:Count
    rC = g_rC(Res.SimulationOutput.Position(i, :)');
    Res.rC(i, 1) = rC(1);
    Res.rC(i, 2) = rC(3);
end

figure;
plot(Res.SimulationOutput.Time, Res.rC);
title('r_C')


%Plot the output
figure_handle = SimulationEngine.PlotSimulationResults(Res.SimulationOutput, 'P, dP; V, U');

%If need - animate the resulting motion
ToAnimate = true;
if ToAnimate
    Animation = SRDAnimation();
    Animation.Animation_Accelerator = 100;
    Animation.Animate(Res.SimulationOutput.Position);
end
end