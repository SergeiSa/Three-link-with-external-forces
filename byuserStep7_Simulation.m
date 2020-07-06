function byuserStep7_Simulation()
close all;

%Create user interface object for SRD
SRD = SRDuserinterface;

ExternalForcesEngine = SRD.GetExternalForcesEngine();

%Load SimulationEngine and set up the simulation parameters
SimulationEngine = SRD.GetSimulationEngine();

% a = 10;
% w = 10;
%     function f = GetExternalForces()
%         SensorData = SimulationEngine.SensorHandler.ReadCurrentData;
%         t = SensorData.t;
%         
%         f = [a*sin(w*t); 
%              a*sin(w*t + pi/2)];
%     end

% fx = 100;
% fy = 50;
%     function f = GetExternalForces()
%         SensorData = SimulationEngine.SensorHandler.ReadCurrentData;
%         t = SensorData.t;
%         
%         if (t > 0.3) && (t < 0.7)
%             f = [fx; fy];
%         else
%             f =[0; 0];
%         end
%     end

w = 5;
fx = 100;
fy = 50;
phi0 = 0;
phi1 = pi/6;
    function f = GetExternalForces()
        SensorData = SimulationEngine.SensorHandler.ReadCurrentData;
        t = SensorData.t;
        
        f = [fx* (sin(w*t+phi0)) ./ abs(sin(w*t+phi0));
             fy* (sin(w*t+phi1)) ./ abs(sin(w*t+phi1))];
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
% % InverseKinematicsEngine = SRD.GetInverseKinematicsEngine();
% % SimulationEngine.Time = InverseKinematicsEngine.TimeEnd - 0.0;
% % ControlInput = @InverseKinematicsEngine.EvaluatePolynomialApproximation;

SimulationEngine.Time = 8;
ControlInput = SimulationEngine.GetPlugInput("Constant_IC_ControlInput");

% ControlInput = SimulationEngine.GetPlugInput("Constant_ControlInput", 'value_q', [1.2; -1; 1.2]);


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
% Controller = SimulationEngine.GetPDcontroller('Computed torque PD', 'Kp', eye(SimulationEngine.dof)*500, ...
%                                                                     'Kd', eye(SimulationEngine.dof)*100);

% Controller = SimulationEngine.GetPDcontroller('Varying gains PD', 'Kp', eye(SimulationEngine.dof)*500, ...
%                                                                     'Kd', eye(SimulationEngine.dof)*100);
                                                                
             % Can use .GetPDcontroller with 'PD', 'Varying gains PD

%%%%%%%%%%%%%%%%%
Controller = SimulationEngine.GetLQRcontroller('LQR', 'unified_Q', 10000, 'unified_R', 1, ...
    'ILQR_TimeStep', 0.1);

%%%%%%%%%%%%%%%%%
% Controller = SimulationEngine.GetMPcontroller('MP', 'unified_Q', 10000, 'unified_R', 1, ...
%     'NumberOfPredictionSteps', 10, 'MP_PredictionTimeStep', 0.005);

%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%


    function Output = Tester(~, ~)      
        SensorData = SimulationEngine.SensorHandler.ReadCurrentData;
        rC = g_rC(SensorData.q);   
        
        f = GetExternalForces();
        
        Output.rC = rC;
        Output.f = f;
    end


%Simulate
tic
Res = SimulationEngine.Simulation(ControlInput, Controller, 'Tester', @Tester);
toc

Count = size(Res.SimulationOutput.Position, 1);
Res.rC = zeros(Count, 2);
for i = 1:Count
    %C = g_rC(Res.SimulationOutput.Position(i, :)');
    rC = Res.TesterOutput{i}.rC;
    f = Res.TesterOutput{i}.f;
    Res.rC(i, 1) = rC(1);
    Res.rC(i, 2) = rC(3);
    
    Res.f(i, :) = reshape(f, [1, 2]);
end

figure('Color', 'w', 'Name', 'r_C');
plot(Res.SimulationOutput.Time, Res.rC, 'LineWidth', 3);
% title('r_C')
grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 18;
xlabel_handle = xlabel('$$t$$, s');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$x_C$$ (m), $$y_C$$ (m)');
ylabel_handle.Interpreter = 'latex';
legend_handle = legend('$$x_C$$ (m)', '$$y_C$$ (m)');
legend_handle.Interpreter = 'latex';


figure('Color', 'w', 'Name', 'ExternalForce');
plot(Res.SimulationOutput.Time, Res.f, 'LineWidth', 3);
% title('ExternalForce')
grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 18;
xlabel_handle = xlabel('$$t$$, s');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$f^e_x$$ (N), $$f^e_y$$ (N)');
ylabel_handle.Interpreter = 'latex';
legend_handle = legend('$$f^e_x$$ (N)', '$$f^e_y$$ (N)');
legend_handle.Interpreter = 'latex';


%Plot the output
figure_handle = SimulationEngine.PlotSimulationResults(Res.SimulationOutput, 'P, dP; V, U');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot q1(t)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Color', 'w', 'Name', 'q1');
plot(Res.SimulationOutput.Time, Res.SimulationOutput.Position(:, 1)*(180 / pi), 'LineWidth', 3);
% title('ExternalForce')
grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 18;
xlabel_handle = xlabel('$$t$$, s');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$q_1$$ (grad)');
ylabel_handle.Interpreter = 'latex';
% legend_handle = legend('$$f^e_x$$ (N), $$f^e_y$$ (N)');
% legend_handle.Interpreter = 'latex';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot q2(t)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Color', 'w', 'Name', 'q1');
plot(Res.SimulationOutput.Time, Res.SimulationOutput.Position(:, 1)*(180 / pi), 'LineWidth', 3);
% title('ExternalForce')
grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 18;
xlabel_handle = xlabel('$$t$$, s');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$q_2$$ (grad)');
ylabel_handle.Interpreter = 'latex';
% legend_handle = legend('$$f^e_x$$ (N), $$f^e_y$$ (N)');
% legend_handle.Interpreter = 'latex';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot q3(t)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Color', 'w', 'Name', 'q1');
plot(Res.SimulationOutput.Time, Res.SimulationOutput.Position(:, 3)*(180 / pi), 'LineWidth', 3);
% title('ExternalForce')
grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 18;
xlabel_handle = xlabel('$$t$$, s');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$q_3$$ (grad)');
ylabel_handle.Interpreter = 'latex';
% legend_handle = legend('$$f^e_x$$ (N), $$f^e_y$$ (N)');
% legend_handle.Interpreter = 'latex';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot e_i(t)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Color', 'w', 'Name', 'e_i = q_desired_i - q_i');
plot(Res.SimulationOutput.Time, ...
    ( Res.SimulationOutput.DesiredPosition(:, 1) - Res.SimulationOutput.Position(:, 1) )*(180 / pi), 'LineWidth', 3); hold on;
plot(Res.SimulationOutput.Time, ...
    ( Res.SimulationOutput.DesiredPosition(:, 2) - Res.SimulationOutput.Position(:, 2) )*(180 / pi), 'LineWidth', 3);
plot(Res.SimulationOutput.Time, ...
    ( Res.SimulationOutput.DesiredPosition(:, 3) - Res.SimulationOutput.Position(:, 3) )*(180 / pi), 'LineWidth', 3);
% title('ExternalForce')
grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 18;
xlabel_handle = xlabel('$$t$$, s');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$e_i$$ (grad)');
ylabel_handle.Interpreter = 'latex';
legend_handle = legend('$$e_1$$ (grad)', '$$e_2$$ (grad)', '$$e_3$$ (grad)');
legend_handle.Interpreter = 'latex';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot w_i(t)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Color', 'w', 'Name', 'w_i');
plot(Res.SimulationOutput.Time, ...
    Res.SimulationOutput.Velocity(:, 1), 'LineWidth', 3); hold on;
plot(Res.SimulationOutput.Time, ...
    Res.SimulationOutput.Velocity(:, 2), 'LineWidth', 3);
plot(Res.SimulationOutput.Time, ...
    Res.SimulationOutput.Velocity(:, 3), 'LineWidth', 3);
% title('ExternalForce')
grid on; grid minor;
ax = gca;
ax.GridAlpha = 0.6;
ax.LineWidth = 0.5;
ax.MinorGridLineStyle = '-';
ax.MinorGridAlpha = 0.2;
ax.FontName = 'Times New Roman';
ax.FontSize = 18;
xlabel_handle = xlabel('$$t$$, s');
xlabel_handle.Interpreter = 'latex';
ylabel_handle = ylabel('$$\dot q_i$$ (rad/s)');
ylabel_handle.Interpreter = 'latex';
legend_handle = legend('$$\dot q_1$$ (rad/s)', '$$\dot q_2$$ (rad/s)', '$$\dot q_3$$ (rad/s)');
legend_handle.Interpreter = 'latex';


%If need - animate the resulting motion
ToAnimate = false;
if ToAnimate
    Animation = SRDAnimation();
    Animation.Animation_Accelerator = 100;
    Animation.Animate(Res.SimulationOutput.Position);
end
end