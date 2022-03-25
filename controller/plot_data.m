%% Load the data and plot:
% Plot
N = [10,20,50];
for trial=1:3
    % Load the data
    load(strcat("experiment_",num2str(trial),"_data.mat"));
    % Plot x axis error
    figure(1);
    plot(x_error(1:1900),'DisplayName',"N = "+N(trial));
    ylabel('Error in x')
    xlabel('Timestep')
    hold on
    legend()
    % Plot y axis error
    figure(2);
    hold on;
    plot(y_error(1:1900),'DisplayName',"N = "+N(trial));
    ylabel('Error in y')
    xlabel('Timestep')
    legend()
    % Plot theta error
    figure(3);
    hold on;
    plot(theta_error(1:1900),'DisplayName',"N = "+ N(trial));
    ylabel('Error in heading theta')
    xlabel('Timestep')
    legend()
end