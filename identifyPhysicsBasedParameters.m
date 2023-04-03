%% IDENTIFYPHYSICSBASEDPARAMETERS.M
% This model performs the identification of the physical parameters based
% on the LIP model by minimization of the MSE data fit:
% theta_PGstar = phi_PG\output'. 
% Specialized optimizations can be inserted here, e.g., by fixing specific
% parameters. 
%
% theta_PGstar = identifyPhysicsBasedParameters(phi, output, Ts,
%       typeOfTransform).
% OUTPUT:
%   * theta_PGstar: physical parameters when identifying using the MSE data 
%     fit with the physical (LIP) model only. 
% INPUTS:
%   * phi: the regressor points, i.e., [phi(0), ..., phi(N-1)];
%   * output: the target output, e.g., [u(0), ..., u(N-1)] for direct
%     inverse ID;
%   * Ts: sampling time;
%   * typeOfTransform: choice for the type of NN input transform and
%     physical model.
% 
%--------------------------------------------------------------------------
% Author:   Max Bolderman,
% Contact:  m.bolderman@tue.nl.
% Affiliation: Control Systems Group, Eindhoven University of Technology. 
%--------------------------------------------------------------------------

function theta_PGstar = identifyPhysicsBasedParameters(phi, output, Ts, typeOfTransform)
[~, phi_PGstar] = PGNN_PGT(phi, Ts, typeOfTransform);

if (size(phi_PGstar,1) == 0)    % A NN structure is used, so there are no physical parameters
    theta_PGstar = [];
elseif (typeOfTransform == 1)   % For identification, we fix the linear, position dependency to zero, to ensure robust extrapolation
    theta_PGstar = pinv(phi_PGstar([1,2,4],:)')*output';
    theta_PGstar = [theta_PGstar(1:2); 0; theta_PGstar(3)];
else
    theta_PGstar = pinv(phi_PGstar')*output';
end

