%% PGNN_INITIALIZE.M
% This model constructs the PGNN and initializes its parameters theta^(0). 
% If requested, it updates theta_L = [W_{l+1}, B_{l+1}, theta_phy]
% according to the optimized initialization. 
%
% [theta_0, n_params] = PGNN_Initialize(output, phi, phi_E, ...
%       Ts, typeOfTransform, theta_PGstar, networkSize, lambda, ...
%       reg_params, useInitialization)
% OUTPUTS:
%   * theta_0: initialized PGNN parameters;
%   * n_params: number of paramers in the network weights, biases, and 
%     physical model.
% INPUTS:
%   * output: target output, e.g., [u(0), ..., u(N-1)] for direct inverse
%     ID; 
%   * phi: the regressor points, i.e., [phi(0), ..., phi(N-1)];
%   * phi_E: regressor points for extrapolation data; 
%   * Ts: sampling time;
%   * typeOfTransform: choice for the type of NN input transform and 
%     physical model;
%   * theta_PGstar: physical parameters when identifying using the MSE data 
%     fit with the physical (LIP) model only;
%   * networkSize: dimensions of network, i.e., [n_1, ..., n_l];
%   * lambda: regularization parameter;
%   * reg_params: regularization parameter {lambda_phy, lambda_NN, ...
%     gamma_ZN, gamma_ZE};
%   * useInitialization: boolean choice whether to use the optimized 
%     initialization.
% 
%--------------------------------------------------------------------------
% Author:   Max Bolderman,
% Contact:  m.bolderman@tue.nl.
% Affiliation: Control Systems Group, Eindhoven University of Technology. 
%--------------------------------------------------------------------------

function [theta_0, n_params] = PGNN_Initialize(output, phi, phi_E, Ts, typeOfTransform, theta_PGstar, networkSize, lambda, reg_params, useInitialization)

factorParasitics = 1000;  % Initialize the NN output weights and bias with a factorParasitics time smaller

% Compute phi_PG and phi_NN for dimensioning
[phi_NN, phi_PG]    = PGNN_PGT(phi, Ts, typeOfTransform);

% Find the number of parameters that are within the (PG)NN
n_params            = zeros(1, 2*size(networkSize,2)+2+1);  % [N_{w1}, N_{b1}, ... , N_{W_{l+1}}, N_{B_{l+1}}, N_{theta_PG}]
n_params(1)         = size(phi_NN,1)*networkSize(1);
n_params(2)         = networkSize(1);
n_params(end-2)     = networkSize(end)*1;
n_params(end-1)     = 1;
n_params(end)       = size(phi_PG,1);
for ii = 2:size(networkSize,2)
    n_params((ii-1)*2+1) = networkSize(ii)*networkSize(ii-1);
    n_params((ii-1)*2+2) = networkSize(ii);
end

% Initialize the parameter vector with zeros and fill in accordlingly
theta_0 = zeros(sum(n_params),1);
% Compute input-output normalization
min_phiNN = min(phi_NN, [], 2); max_phiNN = max(phi_NN, [], 2);
offset = (min_phiNN + max_phiNN)/2; factor = max_phiNN-offset;

W1 = rand(networkSize(1), size(phi_NN,1))*2-1; B1 = rand(networkSize(1), 1)*2-1;
%W1 = rands(networkSize(1), size(phi_NN,1)); B1 = rands(networkSize(1));
B1 = B1 - W1*diag(1./factor)*offset; W1 = W1*diag(1./factor); W1 = reshape(W1, [], 1);
theta_0(1:n_params(1)) = W1;
theta_0(n_params(1)+1:n_params(1)+n_params(2)) = B1;

for ii = 2:1:size(networkSize, 2)
    Wii = rand(networkSize(ii), networkSize(ii-1))*2-1; Wii = reshape(Wii, [], 1);
    Bii = rand(networkSize(ii), 1)*2-1;
    %Wii = rands(networkSize(ii), networkSize(ii-1)); Wii = reshape(Wii, [], 1);
    %Bii = rands(networkSize(ii), 1);
    indices = [sum(n_params(1:(ii-1)*2))+1:sum(n_params(1:(ii-1)*2+2))];
    theta_0(indices) = [Wii; Bii];
end
Wl1 = rand(1, networkSize(end))*2-1; 
Bl1 = rand(1, 1)*2-1;
%Wl1 = rands(1, networkSize(end));
%Bl1 = rands(1, 1);
min_output = min(output, [], 2); max_output = max(output, [], 2);
offset = (min_output + max_output)/2; factor = max_output-offset;
Wl1 = diag(factor)*Wl1; Bl1 = diag(factor)*Bl1+offset;
Wl1 = reshape(Wl1, [], 1);
    
if (n_params(end) == 0)     % A (PI)NN is used
    indices = [sum(n_params(1:end-3))+1:sum(n_params(1:end))];
    theta_0(indices) = [Wl1; Bl1; theta_PGstar];
else                        % A PGNN is used
    indices = [sum(n_params(1:end-3))+1:sum(n_params(1:end))];
    theta_0(indices) = [Wl1./factorParasitics; Bl1./factorParasitics; theta_PGstar];
end

% Adjust initialization using the optimized initialization
if (useInitialization == 1)
    [~, xi] = NN_Output(phi_NN, theta_0(1:end-n_params(end)), networkSize, n_params(1:end-1));
    
    % Define dimensions
    N_data = size(output,2);
    N_phy  = n_params(end);
    N_HL   = n_params(end-2);
    N_E    = size(phi_E,2);
            
    % Add data fit
    phi_OL = [xi; ones(1, size(xi,2)); phi_PG];
    B = [1/sqrt(N_data)*output'];        
    A = [1/(sqrt(N_data))*phi_OL'];
    % Add regularization for theta_phy
    if (N_phy > 0)
        B = [B;
             sqrt(lambda/N_phy)*reg_params{1}.*theta_PGstar];
        A = [A;
             zeros(N_phy, N_HL+1), sqrt(lambda/N_phy)*reg_params{1}.*eye(N_phy, N_phy)];
    end
    % Add regularization for W_{l+1} and B_{l+1}
    if (size(reg_params{2}, 1) == 1)
        B = [B;
             sqrt(lambda/(N_HL+1))*reg_params{2}.*zeros(N_HL+1, 1)];
        A = [A;
             sqrt(lambda/(N_HL+1))*reg_params{2}.*eye(N_HL+1, N_HL+1), zeros(N_HL+1, N_phy)];
    else
        B = [B;
             sqrt(lambda/(N_HL+1))*reg_params{2}(end-N_HL:end).*zeros(N_HL+1,1)];
        A = [A;
             sqrt(lambda/(N_HL+1))*reg_params{2}(end-N_HL:end).*eye(N_HL+1, N_HL+1), zeros(N_HL+1, N_phy)];
    end
    % Add PINNs regularization for Z^N
    if (reg_params{3} > 0)
        B = [B; 
             sqrt(reg_params{3}/(N_data)).*PG_ModelOutput(phi, Ts, theta_PGstar, typeOfTransform)'];
        A = [A;
             sqrt(reg_params{3}/(N_data)).*phi_OL'];
    end
    % Add PINNs regularization for Z^N
    if (N_E == 0) else
        [phi_NNE, phi_PGE]    = PGNN_PGT(phi_E, Ts, typeOfTransform);
        [~, xiE] = NN_Output(phi_NNE, theta_0(1:end-n_params(end)), networkSize, n_params(1:end-1));
    end
    if (reg_params{4} > 0) && (N_E > 0)
        phi_OLE = [xiE; ones(1, size(xiE,2)); phi_PGE];
        B = [B;
             sqrt(reg_params{4}/(N_E)).*PG_ModelOutput(phi_E, Ts, theta_PGstar, typeOfTransform)'];
        A = [A;
             sqrt(reg_params{4}/(N_E)).*phi_OLE'];
    end
    
    % Compute the optimized parameter values
    theta_OL            = A\B;    
    % Replace output weights with optimized ones
    theta_0(indices)    = theta_OL;
end





