%% Compare how N changes the error with terminal cost
% Plot
clear;
clc;
N = [5,10,20,50];
xlimit = 900;
t = linspace(0,10,1000);
for trial=1:4
    % Load the data
    load(strcat("experiment_horizon_",num2str(trial),"_data.mat"));
    figure(1);
    hold on;
    xy_error = abs(x_error(1:xlimit)) + abs(y_error(1:xlimit));
    plot(t(1:xlimit),xy_error(1:xlimit),'DisplayName',"N = "+N(trial),'LineWidth',1);
    ylabel('Total error in X-Y [m]')
    xlabel('Time [s]')
    legend()
    title('Effect of horizon N on error (with terminal cost)')
    grid on
end
exportgraphics(figure(1),'Effect of horizon N on error (with terminal cost).jpg','Resolution',720)
%% Compare how N changes the error without terminal cost
clear;
clc;
N = [5,10,20,50];
xlimit = 900;
t = linspace(0,10,1000);
for trial=1:4
    % Load the data
    load(strcat("experiment_horizon_no_Vf",num2str(trial),"_data.mat"));
    figure(1);
    hold on;
    xy_error = abs(x_error(1:xlimit)) + abs(y_error(1:xlimit));
    plot(t(1:xlimit),xy_error(1:xlimit),'DisplayName',"N = "+N(trial),'LineWidth',1);
    ylabel('Total absolute error in X and Y [metres]')
    xlabel('Time [s]')
    legend()
    title('Effect of horizon N on error (without terminal cost)')
    grid on
end
exportgraphics(figure(1),'Effect of horizon N on error (without terminal cost).jpg','Resolution',720)
%% Compare oldcost vs newcost control input for N=10:
% Plot control input
clear;
clc;
load(strcat("experiment_uref_",num2str(1),"_data.mat"));
xlimit = 900;
t = linspace(0,10,1000);
trial = 2;
load(strcat("experiment_newcost_control_",num2str(trial),"_data.mat"));
figure(1);
hold on;
control_input = sum(abs(uStore-uref(1:990,:)),2);
plot(t(1:xlimit),control_input(1:xlimit),'DisplayName',"Control input with Terminal cost",'LineWidth',1);
ylabel('Total control input u')
xlabel('Time [s]')
grid on
legend()


load(strcat("experiment_oldcost_control_",num2str(trial),"_data.mat"));
figure(1);
control_input = sum(abs(uStore-uref(1:990,:)),2);
plot(t(1:xlimit),control_input(1:xlimit),'DisplayName',"Control input without Terminal cost",'LineWidth',1);
ylabel('Total control input u')
xlabel('Time [s]')
grid on
legend()

%% For comparison between with terminal cost and without N=10:
% Plot
clear;
clc;
t = linspace(0,10,1000);
xlimit = 900;
label = {'With Terminal Cost';'Without Terminal Cost'};
for trial=1:2 
    % Load the data
    load(strcat("experiment_",num2str(trial),"_data.mat"));

    figure(1);
    hold on;
    xy_error = abs(x_error(1:900)) + abs(y_error(1:900));
    plot(t(1:xlimit),xy_error(1:900),'DisplayName',label{trial},'LineWidth',1);
    ylabel('Total error in X-Y [m]')
    xlabel('Time [s]')
    legend()
    grid on
    title('Effect of terminal cost on error (N = 10)')
end

exportgraphics(figure(1),'Effect of terminal cost on error (N = 10).jpg','Resolution',720)
%% Compare beta*terminal_cost effects for N=10
clear;
clc;
beta = [0.1,1,2,5];
xlimit = 500;
t = linspace(0,10,1000);
load(strcat("experiment_uref_",num2str(1),"_data.mat"));
for trial=1:4
    % Load the data
    load(strcat("experiment_beta_",num2str(trial),"_data.mat"));
    figure(1);

    subplot(2,1,1);
    hold on;
    xy_error = abs(x_error(1:xlimit)) + abs(y_error(1:xlimit));
    plot(t(1:xlimit),xy_error(1:xlimit),'DisplayName',strcat("Beta = ",num2str(beta(trial))),'LineWidth',1);
    ylabel('Total error in X-Y [m]')
    xlabel('Time [s]')
    grid on
    legend()
    title('Effect of terminal cost scaling beta on error (N = 10)')

    subplot(2,1,2);
    hold on;
    control_input = sum(abs(uStore-uref(1:990,:)),2);
    plot(t(1:xlimit),control_input(1:xlimit),'DisplayName',strcat("Beta = ",num2str(beta(trial))),'LineWidth',1);
    ylabel('Total control input u')
    xlabel('Time [s]')
    grid on
    legend()
    title('Effect of terminal cost scaling beta on total control input u (N = 10)')
end

exportgraphics(figure(1),'Effect of terminal cost scaling beta on error (N = 10).jpg','Resolution',720)
%exportgraphics(figure(2),'Effect of terminal cost scaling beta on total control input u (N = 10).jpg','Resolution',720)
%% Check LQR
clear;
clc;
xlimit = 900;
t = linspace(0,10,1000);
load(strcat("experiment_LQR_",num2str(1),"_data.mat"));
figure(1);
control_input = (uStore-uref(1:990,:));



subplot(3,1,1);
hold on
xy_error = abs(x_error(1:xlimit)) + abs(y_error(1:xlimit));
plot(t(1:xlimit),xy_error,'DisplayName',"LQR",'LineWidth',1);
subplot(3,1,2);
hold on
plot(t(1:xlimit),control_input(1:xlimit,1),'DisplayName',"LQR",'LineWidth',1);
subplot(3,1,3);
hold on
plot(t(1:xlimit),control_input(1:xlimit,2),'DisplayName',"LQR",'LineWidth',1);

legend()
% N = 10
load(strcat("experiment_LQR_",num2str(2),"_data.mat"));
control_input = (uStore-uref(1:990,:));


subplot(3,1,1);
xy_error = abs(x_error(1:900)) + abs(y_error(1:900));
plot(t(1:xlimit),xy_error,'DisplayName',"MPC",'LineWidth',1);
ylabel('Total error in X-Y [m]')
xlabel('Time [s]')
title('Tracking performance of uncostrained LQR vs MPC')
grid on
legend()

subplot(3,1,2);
plot(t(1:xlimit),control_input(1:xlimit,1),'DisplayName',"MPC",'LineWidth',1);
ylabel('Total input v [m/s]')
xlabel('Time [s]')
title('Control performance of uncostrained LQR vs MPC')
grid on
legend()

subplot(3,1,3);
plot(t(1:xlimit),control_input(1:xlimit,2),'DisplayName',"MPC",'LineWidth',1);
ylabel('Total input w [rad/s]')
xlabel('Time [s]')
title('Control performance of uncostrained LQR vs MPC')
grid on
legend()





exportgraphics(figure(1),'Performance of uncostrained LQR vs MPC.jpg','Resolution',720)
%%
clear;
clc;
load("terminal_cost_decrease_data.mat");
subplot(2,1,1);
t = 0:1:length(stage_costs)-1;

xlimit = 500;

plot(t(1:xlimit),Vf_s(1:xlimit),'DisplayName',"Terminal cost $V_f(x,i)$",'LineWidth',1);
ylabel('Cost')
xlabel('Time step i')
title('Terminal cost across time')
grid on
leg1 = legend('show');
set(leg1, 'Interpreter','latex')
set(gca,'fontsize',12)

subplot(2,1,2);
plot(t(1:xlimit),Vf_diffs(1:xlimit),'DisplayName',"Terminal cost decrease $V_f(x,i) - V_f(f(x,u)$,i)",'LineWidth',1);
hold on;
plot(t(1:xlimit),stage_costs(1:xlimit),'DisplayName',"Stage cost $\ell(x,u,i)$",'LineWidth',1);
ylabel('Cost')
xlabel('Time step i')
title('Terminal cost decrease and Stage cost across time')
grid on
leg2 = legend('show');
set(leg2, 'Interpreter','latex')
set(gca,'fontsize',12)
exportgraphics(figure(1),'Terminal_cost_decrease.jpg','Resolution',720)