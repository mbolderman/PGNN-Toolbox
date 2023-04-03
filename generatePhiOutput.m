%% GENERATEPHIOUTPUT.M 
% This model constructs the matrix phi and the 
% target output based on an input-output data set,
% the order of the dynamics, and the type of identification. 
% DIRECT INVERSE IDENTIFICATION:
%   * phi = [phi(0), ..., phi(N-1)], phi(t) = [y(t+n+k+1), ...,
%     y(t+n_k-n_a+1), u(t-1), ..., u(t-n_b+1)]^T;
%   * output = [u(0), ..., u(N-1)]. 
% FORWARD IDENTIFICATION:
%   * phi = [phi(0), ..., phi(N-1)], phi(t) = [y(t-1), ..., y(t-n_a),
%     u(t-n_k-1), ..., u(t-n_k-n_b)]^T;
%   * output = [y(0), ..., y(N-1)].
%
% [phi, output, phi_val, output_val, phi_E] =
%       generatePhiOutput(u, y, orders, typeOfInverse, partValData, ...,
%       reg_params, typeOfTransform, Ts).
% OUTPUTS: 
%   * phi: the regressor points, i.e., [phi(0), ..., phi(N-1)];
%   * output: the target output, e.g., [u(0), ..., u(N-1)] for direct
%     inverse ID; 
%   * phi_val: regressor points for validation data;
%   * output_val: target output for validation data;
%   * output_E: target data for extrapolation data;
% INPUTS: 
%   * u: input data, i.e., [u(0), ..., u(N-1)];
%   * y: output data, i.e., [y(0), ..., y(N-1)];
%   * orders: order of dynamics, i.e., [n_a, n_b, n_k];
%   * typeOfInverse: choice for the type of inversion method to be used;
%   * partValData: part of the data to be used for validation;
%   * reg_params: regularization parameter {lambda_phy, lambda_NN, ...
%     gamma_ZN, gamma_ZE};
%   * typeOfTrainsform: choice for the type of NN input transform and
%     physical model;
%   * Ts: sampling time. 
%
%--------------------------------------------------------------------------
% Author:   Max Bolderman,
% Contact:  m.bolderman@tue.nl.
% Affiliation: Control Systems Group, Eindhoven University of Technology. 
%--------------------------------------------------------------------------

function [phi, output, phi_val, output_val, phi_E] = generatePhiOutput(u, y, orders, typeOfInverse, partValData, reg_params, Ts)
n_a = orders(1); n_b = orders(2); n_k = orders(3);
if (typeOfInverse == 0) || (typeOfInverse == 2)     % Use a direct inverse identification
    k = max(n_a-1-n_k, n_b-1)+1:1:size(u,2)-(n_k+1);
    output = u(k);
    phi = zeros(n_a+n_b, size(k,2));
    for (ii=1:1:n_a+1)
        phi(ii,:) = y(k+n_k+2-ii);
    end
    for (ii=1:1:n_b-1)
        phi(ii+n_a+1,:) = u(k-ii);
    end
elseif (typeOfInverse == 1) || (typeOfInverse == 3)     % Use a forward identification
    k = max(n_k+n_b,n_a)+1:1:size(u,2);
    output = y(k); 
    phi = zeros(n_a+n_b, size(k,2));
    for (ii=1:1:n_a)
        phi(ii,:) = y(k-ii);
    end
    for (ii=1:1:n_b)
        phi(ii+n_a,:) = u(k-n_k-ii);
    end
else
    fprintf('Incorrect input for typeOfInverse.\n');
end


% Define entries used for training and validation
numberOfValPoints   = round(size(phi,2)*partValData);
entriesValPoints    = randperm(size(phi,2), numberOfValPoints);
entriesTraPoints    = true(1, size(phi,2)); 
entriesTraPoints(entriesValPoints) = false;
% Define validation and training sets
phi_val             = phi(:, entriesValPoints);
output_val          = output(:, entriesValPoints);
phi                 = phi(:, entriesTraPoints);
output              = output(:, entriesTraPoints);

if (reg_params{4} > 0)
    
    %% Here the extrapolation data is made by gridding the space. Extension can be added by computing the cost function
    acc = linspace(-5, 5, 20);
    vel = linspace(-0.2, 0.2, 20);
    pos = [linspace(-0.18, -0.11, 10), linspace(0.11, 0.18, 10)];
    phi_ext = zeros(3, size(acc,2)*size(vel,2)*size(pos,2));
    for ii = 1:1:size(acc,2)
        for jj = 1:1:size(vel,2)
            for kk = 1:1:size(pos,2)
                phi_ext(:, (ii-1)*size(vel,2)*size(pos,2)+(jj-1)*size(pos,2)+kk) = [pos(kk), vel(jj), acc(ii)];
            end
        end
    end
    if (typeOfInverse == 0)
        phi_E = zeros(n_a+n_b, size(phi_ext,2));
        for (ii=1:1:n_a+1)
            DeltaT = Ts*(ii-1);
            phi_E(ii,:) = phi_ext(1,:) - phi_ext(2,:)*DeltaT - 0.5*phi_ext(3,:)*DeltaT^2;
        end
    end
    phi_E = [];
else
    phi_E = [];
end


