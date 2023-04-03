%% NN_ACTIVATIONFUNCTION.M
% This model computes the activation function in the NN:
%     x_i = alpha_i (zeta_i), with x_i the output of layer i, and zeta_i
%     the input to layer i, i.e., zeta_i = W_i x_{i-1} + B_i. 
%
% Different activation functions can be chosen by uncommenting, or creating
% a desired activation function. 
%
% [x_i] = NN_ActivationFunction(x_imin1).
% OUTPUT:
%   * x_i: output of layer i.
% INPUTS:
%   * zeta_i: input to layer i. 
%
%--------------------------------------------------------------------------
% Author:   Max Bolderman,
% Contact:  m.bolderman@tue.nl.
% Affiliation: Control Systems Group, Eindhoven University of Technology. 
%--------------------------------------------------------------------------
function [x_i] = NN_ActivationFunction(zeta_i)
x_i = tanh(zeta_i);         % <- tanh activation function
% x_i = max(zeta_i, 0);     % <- ReLU activation function
% x_i = exp(-zeta_i.^2);    % <- Gaussian Radial Basis Function



