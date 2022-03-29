%% Compare how N changes the error with terminal cost
% Plot
clear;
clc;
N = [5,10,20,50];
xlimit = 500;
for trial=1:4
    % Load the data
    load(strcat("experiment_horizon_",num2str(trial),"_data.mat"));
%     % Plot x axis error
%     figure(1);
%     plot(x_error(1:xlimit),'DisplayName',"N = "+N(trial));
%     ylabel('Error in x')
%     xlabel('Timestep')
%     hold on
%     legend()
%     % Plot y axis error
%     figure(2);
%     hold on;
%     plot(y_error(1:xlimit),'DisplayName',"N = "+N(trial));
%     ylabel('Error in y')
%     xlabel('Timestep')
%     legend()
    % Plot theta error
%     figure(3);
%     hold on;
%     plot(theta_error(1:1900),'DisplayName',"N = "+ N(trial));
%     ylabel('Error in heading theta')
%     xlabel('Timestep')
%     legend()
    figure(1);
    hold on;
    xy_error = abs(x_error(1:xlimit)) + abs(y_error(1:xlimit));
    plot(xy_error(1:xlimit),'DisplayName',"N = "+N(trial),'LineWidth',1);
    ylabel('Total absolute error in X and Y')
    xlabel('Timestep')
    legend()
end
%% Compare how N changes the error without terminal cost
clear;
clc;
N = [5,10,20,50];
xlimit = 900;
for trial=1:4
    % Load the data
    load(strcat("experiment_horizon_no_Vf",num2str(trial),"_data.mat"));
    figure(4);
    hold on;
    xy_error = abs(x_error(1:xlimit)) + abs(y_error(1:xlimit));
    plot(xy_error(1:xlimit),'DisplayName',"N = "+N(trial),'LineWidth',1);
    ylabel('Total absolute error in X and Y')
    xlabel('Timestep')
    legend()

end
%% Compare oldcost vs newcost control input for N=10:
% Plot control input
clear;
clc;
load(strcat("experiment_uref_",num2str(1),"_data.mat"));
xlimit = 900;
trial = 2;
load(strcat("experiment_newcost_control_",num2str(trial),"_data.mat"));
figure(1);
hold on;
control_input = sum(abs(uStore-uref(1:990,:)),2);
plot(control_input(1:xlimit),'DisplayName',"Control input with Terminal cost",'LineWidth',1);
ylabel('Total control input')
xlabel('Timestep')
legend()

load(strcat("experiment_oldcost_control_",num2str(trial),"_data.mat"));
figure(1);
control_input = sum(abs(uStore-uref(1:990,:)),2);
plot(control_input(1:xlimit),'DisplayName',"Control input without Terminal cost",'LineWidth',1);
ylabel('Total control input')
xlabel('Timestep')
legend()

%% For comparison between with terminal cost and without N=10:
% Plot
clear;
clc;
label = {'With Terminal Cost';'Without Terminal Cost'};
for trial=1:2 
    % Load the data
    load(strcat("experiment_",num2str(trial),"_data.mat"));
%     % Plot x axis error
%     figure(1);
%     plot(x_error(1:900),'DisplayName',label{trial});
%     ylabel('Error in x')
%     xlabel('Timestep')
%     hold on
%     legend()
%     % Plot y axis error
%     figure(2);
%     hold on;
%     plot(y_error(1:900),'DisplayName',label{trial});
%     ylabel('Error in y')
%     xlabel('Timestep')
%     legend()
%     % Plot theta error
%     figure(3);
%     hold on;
%     plot(theta_error(1:900),'DisplayName',label{trial});
%     ylabel('Error in heading theta')
%     xlabel('Timestep')
%     legend()
    figure(4);
    hold on;
    xy_error = abs(x_error(1:900)) + abs(y_error(1:900));
    plot(xy_error(1:900),'DisplayName',label{trial},'LineWidth',1);
    ylabel('Total absolute error in X and Y')
    xlabel('Timestep')
    legend()
end
%% Compare beta*terminal_cost effects for N=10
clear;
clc;
beta = [0.1,1,2,5];
xlimit = 500;
load(strcat("experiment_uref_",num2str(1),"_data.mat"));
for trial=1:4
    % Load the data
    load(strcat("experiment_beta_",num2str(trial),"_data.mat"));
    figure(1);
    hold on;
    xy_error = abs(x_error(1:xlimit)) + abs(y_error(1:xlimit));
    plot(xy_error(1:xlimit),'DisplayName',strcat("Total error in X and Y for beta = ",num2str(beta(trial))),'LineWidth',1);
    ylabel('Total absolute error in X and Y')
    xlabel('Timestep')
    legend()

    figure(2);
    hold on;
    control_input = sum(abs(uStore-uref(1:990,:)),2);
    plot(control_input(1:xlimit),'DisplayName',strcat("Control input for beta = ",num2str(beta(trial))),'LineWidth',1);
    ylabel('Total control input')
    xlabel('Timestep')
    legend()
end

%% Check LQR
clear;
clc;
xlimit = 900;
load(strcat("experiment_LQR_",num2str(1),"_data.mat"));
figure(1);
control_input = (uStore-uref(1:990,:));
subplot(2,1,1);
hold on
plot(control_input(1:xlimit,1),'DisplayName',"Control input v for LQR",'LineWidth',1);
legend()
subplot(2,1,2);
hold on
plot(control_input(1:xlimit,2),'DisplayName',"Control input w for LQR",'LineWidth',1);
ylabel('Total control input')
xlabel('Timestep')
legend()
% N = 10
load(strcat("experiment_LQR_",num2str(2),"_data.mat"));
control_input = (uStore-uref(1:990,:));
subplot(2,1,1);
plot(control_input(1:xlimit,1),'DisplayName',"Control input v for MPC",'LineWidth',1);
legend()
subplot(2,1,2);
plot(control_input(1:xlimit,2),'DisplayName',"Control input w for MPC",'LineWidth',1);
ylabel('Total control input')
xlabel('Timestep')
legend()