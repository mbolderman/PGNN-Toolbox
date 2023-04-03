%% PG_MODELOUTPUT.M
% This model computes the output of the physical model by hardcoding the
% physics. 
% When a PGNN is used, it computes the physical model output as:
%       {theta_phy^*}^T T_{phy}(phi(k)).
% When a NN is used, it uses a hardcoded version. 
%
% [PG_output] = PG_ModelOutput(phi, Ts, typeOfTransform, theta_PGstar).
% OUTPUTS:
%   * PG_output: output of the physical model identified solely, or using a
%     hardcoded version in the situation that a NN is used. 
% INPUTS: 
%   * phi: the regressor points, i.e., [phi(0), ..., phi(N-1)];
%   * Ts: sampling time;
%   * theta_PGstar: physical parameters when identifying using the MSE data 
%     fit with the physical (LIP) model only;
%   * typeOfTransform: choice for the type of NN input transform and 
%     physical model. 
% 
%--------------------------------------------------------------------------
% Author:   Max Bolderman,
% Contact:  m.bolderman@tue.nl.
% Affiliation: Control Systems Group, Eindhoven University of Technology. 
%--------------------------------------------------------------------------

function [PG_output] = PG_ModelOutput(phi, Ts, theta_PGstar, typeOfTransform)

if (size(theta_PGstar, 1) > 0) 
    % A PGNN is used with physical model
    [~, phi_PG] = PGNN_PGT(phi, Ts, typeOfTransform);
    PG_output = theta_PGstar'*phi_PG;
else
    % A NN is used, i.e., no physical model
    theta_PG = [18.705204697222810; 1.246199943552432e+02; 0; 8.413373493557593]; 
    phi_PG = [0.5*((phi(1,:)-2*phi(3,:)+phi(5,:))/(4*Ts^2)+(phi(2,:)-2*phi(4,:)+phi(6,:))/(4*Ts^2));      % \delta^2 y(t)
              0.5*((phi(2,:)-phi(4,:))/(2*Ts)+(phi(3,:)-phi(5,:))/(2*Ts));                   % \delta y(t)
              0.5*(phi(3,:)+phi(4,:));                                      % y(t)
              0.5*(sign((phi(2,:)-phi(4,:))/(2*Ts))+sign((phi(3,:)-phi(5,:))/(2*Ts)))]; 
    PG_output = theta_PG'*phi_PG;
end
    
    
    
    
