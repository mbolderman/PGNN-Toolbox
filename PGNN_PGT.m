%% PGNN_PGT.m
% This model computes the NN input transformation and the LIP
% transformation of the physical model:
%   * phi_NN  = T_NN(phi(k));
%   * phi_phy = T_phy(phi(k)). 
% 
% A NN is obtained by choosing phi_phy = [], i.e., there is no physical
% model in the PGNN. 
%
% [phi_NN, phi_phy] = PGNN_PGT(phi, Ts, typeOfTransform).
% OUTPUTS:
%   * phi_NN: regressor that enters the NN;
%   * phi_phy: regressor of the LIP physical model.
% INPUTS:
%   * phi: the regressor points, i.e., [phi(0), ..., phi(N-1)];
%   * Ts: sampling time;
%   * typeOfTransform: choice for the type of NN input transform and
%     physical model. 
%
%--------------------------------------------------------------------------
% Author:   Max Bolderman,
% Contact:  m.bolderman@tue.nl.
% Affiliation: Control Systems Group, Eindhoven University of Technology. 
%--------------------------------------------------------------------------

function [phi_NN, phi_phy] = PGNN_PGT(phi, Ts, typeOfTransform)
if (typeOfTransform == 0)
    phi_NN = phi; 
    phi_phy = phi;
elseif (typeOfTransform == 1)
    % n_a = 5; n_b = 1; n_k = 2;
    phi_NN = [0.5*((phi(1,:)-2*phi(3,:)+phi(5,:))/(4*Ts^2) + (phi(2,:)-2*phi(4,:)+phi(6,:))/(4*Ts^2));      % 0.5 (\delta^2 y(t+1) + \delta^2 y(t))
              0.5*((phi(2,:)-phi(4,:))/(2*Ts) + (phi(3,:)-phi(5,:))/(2*Ts));                                % 0.5 (\delta y(t+1) + \delta y(t))
              0.5*(phi(3,:) + phi(4,:));                                                                    % 0.5 (y(t+1) + y(t)) 
              0.5*(sign((phi(2,:)-phi(4,:))/(2*Ts)) + sign((phi(2,:)-phi(4,:))/(2*Ts)))];                    % 0.5 (sign(\delta y(t+1)) + sign(\delta y(t)))
    phi_phy = phi_NN;
elseif (typeOfTransform == 2)       % Example of PINNs -> Put phi_PG empty and hardcode the physical model in PG_ModelOutput
    % n_a = 5; n_b = 1; n_k = 2;
    phi_NN = [0.5*((phi(1,:)-2*phi(3,:)+phi(5,:))/(4*Ts^2) + (phi(2,:)-2*phi(4,:)+phi(6,:))/(4*Ts^2));      % 0.5 (\delta^2 y(t+1) + \delta^2 y(t))
              0.5*((phi(2,:)-phi(4,:))/(2*Ts) + (phi(3,:)-phi(5,:))/(2*Ts));                                % 0.5 (\delta y(t+1) + \delta y(t))
              0.5*(phi(3,:) + phi(4,:));                                                                    % 0.5 (y(t+1) + y(t)) 
              0.5*(sign((phi(2,:)-phi(4,:))/(2*Ts)) + sign((phi(2,:)-phi(4,:))/(2*Ts)))];                    % 0.5 (sign(\delta y(t+1)) + sign(\delta y(t)))
    phi_phy = [];
elseif (typeOfTransform == 3)       % Black-box NN
    phi_NN = phi; 
    phi_phy = [];
else
    fprintf('Incorrect input for typeOfTransform, no transform of this type is provided.\n');
end

