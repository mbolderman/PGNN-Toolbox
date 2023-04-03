%% PGNN_COSTFUNCTION.M
% This model computes the cost function entries:
%       V (theta, Z^N) = V_{MSE}(theta, Z^N) + lambda V_{reg}(theta) + ...
%          + gamma_{ZN} V_{phy}(theta, Z^N) + gamma_{ZE}V_{phy}(theta, ZE). 
%
% [total_Costs] = PGNN_CostFunction(theta, phi, output, phi_E, ...
%       theta_PGstar, networkSize, n_params, lambda, reg_params, ...
%       Ts, typeOfTransform). 
% OUTPUTS:
%   * total_Costs: column containing all elements in the cost function.
% INPUTS:
%   * theta: PGNN parameters;
%   * phi: the regressor points, i.e., [phi(0), ..., phi(N-1)];
%   * output: target output, e.g., [u(0), ..., u(N-1)] for direct inverse
%     ID; 
%   * phi_E: regressor points for extrapolation data; 
%   * theta_PGstar: physical parameters when identifying using the MSE data 
%     fit with the physical (LIP) model only;
%   * networkSize: dimensions of network, i.e., [n_1, ..., n_l];
%   * n_params: number of paramers in the network weights, biases, and 
%     physical model.
%   * lambda: regularization parameter;
%   * reg_params: regularization parameter {lambda_phy, lambda_NN, ...
%     gamma_ZN, gamma_ZE};
%   * Ts: sampling time;
%   * typeOfTransform: choice for the type of NN input transform and 
%     physical model;
%  
%--------------------------------------------------------------------------
% Author:   Max Bolderman,
% Contact:  m.bolderman@tue.nl.
% Affiliation: Control Systems Group, Eindhoven University of Technology. 
%--------------------------------------------------------------------------

function [total_Costs] = PGNN_CostFunction(theta, phi, output, phi_E, theta_PGstar, networkSize, n_params, lambda, reg_params, Ts, typeOfTransform)
N_data  = size(output,2);
N_phy   = n_params(end);
N_NN    = sum(n_params(1:end-1));
N_E     = size(phi_E,2);

%% Different error components are computed here:
% Data fit
data_Fit = 1/(sqrt(N_data))*transpose(output - PGNN_Output(phi, Ts, typeOfTransform, theta, networkSize, n_params));

% Physical parameters regularization
if (N_phy > 0)
    reg_Phyparam = sqrt(lambda/N_phy)*reg_params{1}.*(theta(sum(n_params(1:end-1))+1:sum(n_params(1:end)))-theta_PGstar);
else
    reg_Phyparam = [];
end
    
% NN parameters regularization
reg_NNparam = sqrt(lambda/N_NN)*reg_params{2}.*theta(1:sum(n_params(1:end-1)));

total_Costs = [data_Fit;
               reg_Phyparam;
               reg_NNparam];

% PINNs regularization (physical model compliance)
if (reg_params{3} == 0)     % No PINNs regularization
else
    output_phy = PG_ModelOutput(phi, Ts, theta_PGstar, typeOfTransform);
    reg_PINN = sqrt(reg_params{3}/N_data).*transpose(output_phy - PGNN_Output(phi, Ts, typeOfTransform, theta, networkSize, n_params));
    total_Costs = [total_Costs;
                   reg_PINN];
end

% PINNs regularization extrapolation
if (reg_params{4} == 0) || (N_E == 0)
else
    output_E = PG_ModelOutput(phi_E, Ts, theta_PGstar, typeOfTransform);
    reg_PINNextr = sqrt(reg_params{4}/N_E).*transpose(output_E - PGNN_Output(phi_E, Ts, typeOfTransform, theta, networkSize, n_params));
    total_Costs = [total_Costs;
                   reg_PINNextr];
end
