%% PGNN_Optimization.M
% This model performs the PGNN optimization by using Matlab's "lsqnonlin"
% optimization:
% 
% [thetahat, res_Costs, res_CostsVal] = PGNN_Optimization(output, phi,...
%       output_val, phi_val, phi_E, Ts, typeOfTransform, theta_0, ...
%       theta_PGstar, networkSize, n_params, lambda, reg_params).
% OUTPUTS: 
%   * thetahat: trained PGNN parameters;
%   * res_Costs: value of the cost function;
%   * res_CostsVal: the cost function evaluated over the validation data;
% INPUTS:
%   * output: target output, e.g., [u(0), ..., u(N-1)] for direct inverse
%     ID; 
%   * phi: the regressor points, i.e., [phi(0), ..., phi(N-1)];
%   * output_val: target output for validation data;
%   * phi_val: regressor points for validation data;
%   * phi_E: regressor points for extrapolation data; 
%   * Ts: sampling time;
%   * typeOfTransform: choice for the type of NN input transform and 
%     physical model;
%   * theta_0: initialized PGNN parameters;
%   * theta_PGstar: physical parameters when identifying using the MSE data 
%     fit with the physical (LIP) model only;
%   * networkSize: dimensions of network, i.e., [n_1, ..., n_l];
%   * n_params: number of paramers in the network weights and biases and 
%     physical model;
%   * lambda: regularization parameter;
%   * reg_params: regularization parameter {lambda_phy, lambda_NN, ...
%     gamma_ZN, gamma_ZE}. 
% NOTE: the current version does not exploit the backpropagation algorithm
% for computing the gradient analytically.
%
%--------------------------------------------------------------------------
% Author:   Max Bolderman,
% Contact:  m.bolderman@tue.nl.
% Affiliation: Control Systems Group, Eindhoven University of Technology. 
%--------------------------------------------------------------------------

function [thetahat, res_Costs, res_CostsVal] = PGNN_Optimization(output, phi, output_val, phi_val, phi_E, Ts, typeOfTransform, theta_0, theta_PGstar, networkSize, n_params, lambda, reg_params)
% theta_hat    -> trained parameters
% res_Costs    -> residual of cost function
% res_CostsVal -> residual of cost for validation data


%% Perform training using lsqnonlin
%options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'MaxFunctionEvaluations', 500*size(theta_0,1), 'MaxIterations', 1000);
options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'Display', 'iter', 'MaxFunctionEvaluations', 500*size(theta_0,1), 'MaxIterations', 1000);
%options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'Display', 'iter', 'MaxFunctionEvaluations', 100*size(theta_0,1), 'MaxIterations', 1000, 'StepTolerance', 1e-12, 'FunctionTolerance', 0);
[thetahat, resNorm] = lsqnonlin(@(theta) PGNN_CostFunction(theta, phi, output, phi_E, theta_PGstar, networkSize, n_params, lambda, reg_params, Ts, typeOfTransform), theta_0, [], [], options);

%% Compute the remaining cost after training
res_Costs       = sum((PGNN_CostFunction(thetahat, phi, output, phi_E, theta_PGstar, networkSize, n_params, lambda, reg_params, Ts, typeOfTransform)).^2);
res_CostsVal    = sum((PGNN_CostFunction(thetahat, phi_val, output_val, [], theta_PGstar, networkSize, n_params, lambda, reg_params, Ts, typeOfTransform)).^2);





