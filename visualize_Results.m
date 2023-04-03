%% VISUALIZE_RESULTS.M
% This model presents the figures that are of interest to visualize,
% validate and observe the effect of the trained PGNN. 
% The following figures are included:
%   1: plot of the PGNN feedforward control signals over time for 
%      reference1.mat for all trained PGNNs;
%   2: plot of the PGNN feedforward control signals over time for 
%      reference2.mat for all trained PGNNs;
%   3: Plot the normalized cost function, data fit, and regularization
%      terms as a function of lambda, if size(lambda,2)>1;
%   4: Plot the L-curve, i.e., plot the regularization cost as a function
%      of the data fit, if size(lambda,2)>1. 
% Setting wantPloti = 1 (0) does (not) present Figure i. 
%
%--------------------------------------------------------------------------
% Author:   Max Bolderman,
% Contact:  m.bolderman@tue.nl.
% Affiliation: Control Systems Group, Eindhoven University of Technology. 
%--------------------------------------------------------------------------


%% Define which plots to provide
wantPlot1 = 1;      % Plots 1, see above
wantPlot2 = 1;      % Plots 2
wantPlot3 = 1;      % Plots 3
wantPlot4 = 1;      % Plots 4


%% 1: FIG. Feedforward over time
%--------------------------------------------------------------------------
while (wantPlot1 == 1) 
    % Load the reference, approximate velocity for figure
    load('reference1'); r = r';
    v    = (r(2:end)-r(1:end-1))/Ts; r = r(1:end-1)'; t = (0:1:size(r,2)-1)*Ts;
    k    = max(1, n_a-n_k):1:size(r,2)-n_k-1;        % Used to access entries of r
    u_ff = zeros(size(lambda,2), size(k,2));
    %% Compute the feedforward action
    for (ii = 1:1:size(lambda,2))
        %% Load the feedforward and create empty matrices to fill in
        %load(['PGNN_ARX_', num2str(networkSize), '_Phi', num2str(typeOfTransform), '_lambda', num2str(ii)]);
        load(['PGNN_ARX_', num2str(ii)]);
        u_ffPG  = zeros(1, size(k,2));
        u_ffNN  = zeros(1, size(k,2));
        phi_ff  = zeros(n_a+n_b, size(k,2));  % Used for computation/memory of the feedforward controller
        %% Compute the feedforward input for all entries required time samples
        % Put in referce values in \phi_ff
        if (typeOfInverse == 0) || (typeOfInverse == 2)
            % The PGNN describes the inverse dynamics
            for (jj = 1:1:n_a+1)
                phi_ff(jj, :) = r(k+n_k+2-jj);
            end
            for (jj = 2:1:size(k,2))
                % Put in past feedforward inputs in \phi_ff
                if (n_b > 1)    
                    phi_ff(n_a+2:n_a+n_b, jj) = [u_ff(ii,jj-1); phi_ff(n_a+2:n_a+n_b-1, jj-1)];
                end
                [u_ff(ii,jj), u_ffPG(jj), u_ffNN(jj)] = PGNN_Output(phi_ff(:,jj), Ts, typeOfTransform, thetahat, networkSize, n_params);
            end
        elseif (typeOfInverse == 1)
            % Should be included here
        end
        %% Plot the results
        figure(); subplot(2,1,1); 
        plot(t(k)-t(k(1)), r(k), 'LineWidth', 2); hold on; grid on; plot(t(k)-t(k(1)), v(k), 'LineWidth', 2);
        ylabel('Reference', 'FontSize', 16, 'Interpreter', 'latex');
        subplot(2,1,2);
        plot(t(k)-t(k(1)), u_ff(ii,:), 'LineWidth', 2); hold on; grid on; plot(t(k)-t(k(1)), u_ffPG); plot(t(k)-t(k(1)), u_ffNN);
        xlabel('Time $[s]$', 'FontSize', 16, 'Interpreter', 'latex'); ylabel('Feedforward $[N]$', 'FontSize', 16, 'Interpreter', 'latex');
        legend({'$u_{\textup{ff}}(t)$', '$u_{\textup{ff,PG}}(t)$', '$u_{\textup{ff,NN}}(t)$'}, 'FontSize', 16, 'Interpreter', 'latex');
        subplot(2,1,1); title(['$\lambda = $', num2str(lambda(ii))], 'FontSize', 20, 'Interpreter', 'latex')
    end
    wantPlot1 = 0;
end


%% 2: FIG. Feedforward over time for extrapolation reference
%--------------------------------------------------------------------------
while (wantPlot2 == 1) 
    % Load the reference, approximate velocity for figure
    load('Reference2'); r = r';
    v = (r(2:end)-r(1:end-1))/Ts; r = r(1:end-1)'; t = (0:1:size(r,2)-1)*Ts;
    %% Compute the feedforward action
    for (ii = 1:1:size(lambda,2))
        %% Load the feedforward and create empty matrices to fill in
        %load(['PGNN_ARX_', num2str(networkSize), '_Phi', num2str(typeOfTransform), '_lambda', num2str(ii)]);
        load(['PGNN_ARX_', num2str(ii)]);
        k       = max(1, n_a-n_k):1:size(r,2)-n_k-1;        % Used to access entries of r
        u_ff    = zeros(1, size(k,2));        % To be filled in with feedforward inputs
        u_ffPG  = zeros(1, size(k,2));
        u_ffNN  = zeros(1, size(k,2));
        phi_ff  = zeros(n_a+n_b, size(k,2));  % Used for computation/memory of the feedforward controller
        %% Compute the feedforward input for all entries required time samples
        % Put in referce values in \phi_ff
        if (typeOfInverse == 0)
            % The PGNN describes the inverse dynamics
            for (jj = 1:1:n_a+1)
                phi_ff(jj, :) = r(k+n_k+2-jj);
            end
            for (jj = 2:1:size(k,2))
                % Put in past feedforward inputs in \phi_ff
                if (n_b > 1)    
                    phi_ff(n_a+2:n_a+n_b, jj) = [u_ff(jj-1); phi_ff(n_a+2:n_a+n_b-1, jj-1)];
                end
                [u_ff(jj), u_ffPG(jj), u_ffNN(jj)] = PGNN_Output(phi_ff(:,jj), Ts, typeOfTransform, thetahat, networkSize, n_params);
            end
        elseif (typeOfInverse == 1)
            % Should be included here
        end
        %% Plot the results
        figure(); subplot(2,1,1); 
        plot(t(k)-t(k(1)), r(k), 'LineWidth', 2); hold on; grid on; plot(t(k)-t(k(1)), v(k), 'LineWidth', 2);
        ylabel('Reference', 'FontSize', 16, 'Interpreter', 'latex');
        subplot(2,1,2);
        plot(t(k)-t(k(1)), u_ff, 'LineWidth', 2); hold on; grid on; plot(t(k)-t(k(1)), u_ffPG); plot(t(k)-t(k(1)), u_ffNN);
        xlabel('Time $[s]$', 'FontSize', 16, 'Interpreter', 'latex'); ylabel('Feedforward $[N]$', 'FontSize', 16, 'Interpreter', 'latex');
        legend({'$u_{\textup{ff}}(t)$', '$u_{\textup{ff,PG}}(t)$', '$u_{\textup{ff,NN}}(t)$'}, 'FontSize', 16, 'Interpreter', 'latex');
        subplot(2,1,1); title(['$\lambda = $', num2str(lambda(ii))], 'FontSize', 20, 'Interpreter', 'latex')
    end
    wantPlot2 = 0;
end



%% 3: FIG. Cost function and regularization terms as a function of lambda
%% 4: FIG. Plot the L curve
%--------------------------------------------------------------------------
while (((wantPlot3 == 1)||(wantPlot4 == 1)) && (size(lambda,2) > 1)) 
    if (size(lambda,2) == 1)
        fprintf(['    No regularization effect plotted, since "size(lambda,2)=1".\n'])
    else
        % Create empty matrices to fill in
        costs_DataFit           = zeros(size(lambda));  % Insert costs for data-fit
        costs_DataFitVal        = zeros(size(lambda));  % Insert costs for data-fit in validation data
        costs_Regularization    = zeros(size(lambda));  % Insert costs for regularization term
        costs_RegularizationVal = zeros(size(lambda));  % Insert costs for regularization term in validation data (note: regularization term for PINNs changes for different phi)
        % Compute the terms of the cost function
        for (ii = 1:1:size(lambda,2))
            % Load the trained controller
            % Costs evaluated over the training data set
            load(['PGNN_ARX_', num2str(ii)]);
            [~, costs_DataFit(ii), cost_Phyparam, cost_NNparam, cost_PINN, cost_PINNextr] = PGNN_CostFunctionCompleteInfo(thetahat, phi, output, [], theta_PGstar, networkSize, n_params, lambda(ii), reg_params, Ts, typeOfTransform);
            yhat = PGNN_Output(phi, Ts, typeOfTransform, thetahat, networkSize, n_params);

            costs_DataFit(ii) = costs_DataFit(ii)+cost_PINN+cost_PINNextr;
            costs_Regularization(ii)    = (cost_Phyparam+cost_NNparam);
            % Costs evaluated over the validation data set
            [~, costs_DataFitVal(ii), cost_Phyparam, cost_NNparam, cost_PINN, cost_PINNextr] = PGNN_CostFunctionCompleteInfo(thetahat, phi_val, output_val, [], theta_PGstar, networkSize, n_params, lambda(ii), reg_params, Ts, typeOfTransform);
            costs_DataFitVal(ii) = costs_DataFitVal(ii)+cost_PINN+cost_PINNextr;
            costs_RegularizationVal(ii)    = (cost_Phyparam+cost_NNparam);
        end
        % Compute the total cost function
        costs_Total     = costs_DataFit + costs_Regularization;         % Value of the cost function evaluated over the training data set
        costs_TotalVal  = costs_DataFitVal + costs_RegularizationVal;   % Value of the cost function evaluated over the validation data set
    end   
    %% Plot the desired figures
    % Plot 3: cost function and regularization term as a function of lambda
    if (wantPlot3 == 1)
        figure(); subplot(2,1,1);
        semilogx(lambda, normalize(costs_Total, 'range'), 'LineWidth', 2); hold on; grid on;
        semilogx(lambda, normalize(costs_DataFit, 'range')); semilogx(lambda, normalize(costs_Regularization./lambda, 'range'));
        ylabel('Norm. costs', 'FontSize', 16, 'Interpreter', 'latex');
        subplot(2,1,2);
        semilogx(lambda, normalize(costs_TotalVal, 'range'), 'LineWidth', 2); hold on; grid on;
        semilogx(lambda, normalize(costs_DataFitVal, 'range')); semilogx(lambda, normalize(costs_RegularizationVal./lambda, 'range'));
        xlabel('$\lambda$', 'FontSize', 16, 'Interpreter', 'latex'); ylabel('Norm. costs', 'FontSize', 16, 'Interpreter', 'latex');
        legend({'$V$', '$V_{\textup{data}}$', '$V_{\textup{reg}}$'}, 'FontSize', 16, 'Interpreter', 'latex');
        subplot(2,1,1); title('Top: training data, Bottom: validation data', 'FontSize', 20, 'Interpreter', 'latex')
        wantPlot3 = 0;
    end
    % Plot 4: L-curve
    if (wantPlot4 == 1)
        figure(); loglog(costs_DataFit, costs_Regularization./lambda, 'LineWidth', 2); hold on; grid on;
        loglog(costs_DataFitVal, costs_RegularizationVal./lambda, 'LineWidth', 2);
        xlabel('Data fit', 'FontSize', 16, 'Interpreter', 'latex'); ylabel('Regularization', 'FontSize', 16, 'Interpreter', 'latex');
        legend({'Training data', 'Validation data'}, 'FontSize', 16, 'Interpreter', 'latex');
        title('L-curve', 'FontSize', 16, 'Interpreter', 'latex');
        wantPlot4 = 0;
    end
end








