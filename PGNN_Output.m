%% PGNN_OUTPUT.M
% This model computes the output of the PGNN:
%   uhat(theta, phi(k)) = theta_phy^T T_phy(phi(k)) ...
%           + f_NN(theta_NN, T_NN(phi(k))). 
% 
% [PGNN_output, PG_output, NN_output] = PGNN_Output(phi, Ts, ...
%       typeOfTransform, theta, networkSize, n_params).
% OUTPUTs:
%   * PGNN_output: output of the PGNN;
%   * PG_output: output of the physical model in the PGNN;
%   * NN_output: output of the NN in the PGNN.
% INPUTS:
%   * phi: the regressor points, i.e., [phi(0), ..., phi(N-1)];
%   * Ts: sampling time;
%   * typeOfTransform: choice for the type of NN input transform and
%     physical model;
%   * theta: PGNN parameters;
%   * networkSize: dimensions of network, i.e., [n_1, ..., n_l];
%   * n_params: number of paramers in the network weights and biases and 
%     physical model.
% 
%--------------------------------------------------------------------------
% Author:   Max Bolderman,
% Contact:  m.bolderman@tue.nl.
% Affiliation: Control Systems Group, Eindhoven University of Technology. 
%--------------------------------------------------------------------------

function [PGNN_output, PG_output, NN_output] = PGNN_Output(phi, Ts, typeOfTransform, theta, networkSize, n_params)

%% Compute the inputs to the NN and the PG part of the PGNN
[phi_NN, phi_PG] = PGNN_PGT(phi, Ts, typeOfTransform);

%% Compute the outputs of the NN and the PG part of the PGNN
if (n_params(end) == 0)     % Note, a NN is used
    PG_output = zeros(1, size(phi,2));
else                        % A PGNN is used
    PG_output = theta(sum(n_params(1:end-1))+1:sum(n_params(1:end)))'*phi_PG;
end
NN_output = NN_Output(phi_NN, theta(1:end-n_params(end)), networkSize, n_params(1:end-1));

%% Combine to compute the total output
PGNN_output = PG_output+NN_output;

