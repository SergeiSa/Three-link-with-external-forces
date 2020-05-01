function Res = u_Simulate_updatetime(a, w, update_time)

%Create user interface object for SRD
SRD = SRDuserinterface;

ExternalForcesEngine = SRD.GetExternalForcesEngine();

%Load SimulationEngine and set up the simulation parameters
SimulationEngine = SRD.GetSimulationEngine();
SimulationEngine.TimeStep = 10^(-4);

    function f = GetExternalForces()
        SensorData = SimulationEngine.SensorHandler.ReadCurrentData;
        t = SensorData.t;
        
        f = [a*sin(w*t); a*sin(w*t + pi/2)];
    end

SimulationEngine.CustomSolverType = 'User-provided';
    function OutputStructure = User_provided_solver()
        
        f = GetExternalForces();
        
        ExternalForcesEngine.UpdateModel(f, SimulationEngine.ModelHandler);
        
        OutputStructure = SimulationEngine.Solver_TaylorUpdate();
    end

SimulationEngine.User_provided_solver = @User_provided_solver;


%Can use 'Euler', 'Taylor', 'Runge', 'Implicit Euler', 'DAE Taylor', 'DAE Runge';
SimulationEngine.IC.q = SimulationEngine.IC.q;
% SimulationEngine.IC.v = SimulationEngine.IC.v + rand(3, 1)*0.5;
SimulationEngine.IC.v = zeros(3, 1);


%Load InverseKinematicsEngine
% InverseKinematicsEngine = SRD.GetInverseKinematicsEngine();

SimulationEngine.Time = 1;

% ControlInput = SimulationEngine.GetPlugInput("Constant_ControlInput", 'value_q', SimulationEngine.IC.q);
ControlInput = SimulationEngine.GetPlugInput("Constant_IC_ControlInput");


%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%
ControllerWrapper = SRDControllerWrapper();
ControllerWrapper.State.u = [];

Kp = eye(SimulationEngine.dof) * 1000;
Kd = eye(SimulationEngine.dof) * 200;


    function PDcontroller(~, ~)
        
        SensorData = SimulationEngine.SensorHandler.ReadCurrentData;
        t = SensorData.t;
        
        if isempty(ControllerWrapper.State.u) || (rem(t, update_time) == 0)
            
            e = SensorData.desired_q - SensorData.q;
            de = SensorData.desired_v - SensorData.v;
            u = Kp*e + Kd*de;
            
            ControllerWrapper.State.u = u;
        else
            u = ControllerWrapper.State.u;
        end
        
        ControllerWrapper.u = u;
    end

ControllerWrapper.Controller = @PDcontroller;

%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%


%     function Tester(~, ~)      
%         SensorData = SimulationEngine.SensorHandler.ReadCurrentData;
%         rC = g_rC(SensorData.q);       
%     end


%Simulate
tic
Res = SimulationEngine.Simulation(ControlInput, ControllerWrapper);
%Can use .Simulation() and .SimulationStateSpace()
toc

Count = size(Res.SimulationOutput.Position, 1);
Res.rC = zeros(Count, 2);
for i = 1:Count
    rC = g_rC(Res.SimulationOutput.Position(i, :)');
    Res.rC(i, 1) = rC(1);
    Res.rC(i, 2) = rC(3);
end
end