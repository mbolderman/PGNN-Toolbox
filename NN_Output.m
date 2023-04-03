%% NN_OUTPUT.M
% This model computes the output of a NN:
%     f_NN(theta_NN, phi_NN(k)) = W_{l+1} alpha_l(...
%           W_2alpha_1(W_1phi_NN(k) + B_1) ...) + B_{l+1}. 
%
% [NN_output, HL_output] = NN_Output(phi_NN, theta_NN, networkSize,
%       n_params). 
% OUTPUTS:
%   * NN_output: output of the NN, i.e., f_NN(theta_NN, phi_NN(k));
%   * HL_output: output of the final hidden layer, i.e., 
%     alpha_l(... W_2 alpha_1(W_1phi_NN(k)+B_1) ...). 
% INPUTS:
%   * phi_NN: regressor that enters the NN;
%   * theta_NN: NN parameters;
%   * networkSize: dimensions of network, i.e., [n_1, ..., n_l];
%   * n_params: number of paramers in the network weights and biases and 
%     physical model.
%
%--------------------------------------------------------------------------
% Author:   Max Bolderman,
% Contact:  m.bolderman@tue.nl.
% Affiliation: Control Systems Group, Eindhoven University of Technology. 
%--------------------------------------------------------------------------

function [NN_output, HL_output] = NN_Output(phi_NN, theta_NN, networkSize, n_params)
% Compute the output of each hidden layer sequentially
for (ii = 1:1:size(networkSize,2))
    if (ii == 1)
        ximin = phi_NN;
        Wi = theta_NN(1:n_params(1)); Wi = reshape(Wi, [networkSize(1), size(phi_NN,1)]);
        Bi = theta_NN(n_params(1)+1:sum(n_params(1:2)));
    else 
        ximin = xi;
        Wi = theta_NN(sum(n_params(1:(ii-1)*2))+1:sum(n_params(1:(ii-1)*2+1))); Wi = reshape(Wi, [networkSize(ii), networkSize(ii-1)]);
        Bi = theta_NN(sum(n_params(1:(ii-1)*2+1))+1:sum(n_params(1:(ii-1)*2+2)));
    end
    xi = NN_ActivationFunction(Wi*ximin+Bi);  % Compute output layer
end
HL_output   = xi;
Wlplus1     = theta_NN(sum(n_params(1:(ii)*2))+1:sum(n_params(1:(ii)*2+1))); Wlplus1 = reshape(Wlplus1, [1, networkSize(end)]);
Blplus1     = theta_NN(sum(n_params(1:(ii)*2+1))+1:sum(n_params(1:(ii)*2+2)));
NN_output   = Wlplus1*xi+Blplus1;


