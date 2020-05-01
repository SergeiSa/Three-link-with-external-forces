close all;

a = 100;
w = [125; 625; 1500];
update_time = 0.01;

figure;
for i = 1:length(w)
Res = u_Simulate_updatetime(a, w(i), update_time);

subplot(1, 2, 1);
plot(Res.SimulationOutput.Time, Res.rC(:, 1)); hold on

subplot(1, 2, 2);
plot(Res.SimulationOutput.Time, Res.rC(:, 2)); hold on
end

