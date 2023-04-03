%% MAIN_PGNN.m 
% This model performs the PGNN training according to the specified
% settings following the structure:
%   1. Load and prepare data set;
%   2. Perform physics-based system identification (currently linear-in-the-parameters);
%   3. (PG)NN identification:
%       3.1. Initialize a (PG)NN with random weights, and optionally perform
%            the optimal initialization. 
%       3.2. Perform training using regularized cost function.
%       3.3. Save the trained (PG)NNs.
%   4. (PG)NN validation:
%       4.1. Show feedforward of trained PGNN.
%       4.2. Show regularization effect (when multiple regularization
%            parameters are inputted) in figures: lambda--MSE(validation),
%            L-curve.
% IMPORTANT: when first running this file, make sure that the variables
% "dataPath" and "fileName" contain the correct path and name of the
% input-output data file.
%
% For more information, see the accompanying paper: M. Bolderman, M. Lazar,
% H. Butler, "A MATLAB toolbox for training and implementing physics-guided
% neural network-based feedforward controllers", 2022. 
%
%--------------------------------------------------------------------------
% Author:   Max Bolderman,
% Contact:  m.bolderman@tue.nl.
% Affiliation: Control Systems Group, Eindhoven University of Technology. 
%--------------------------------------------------------------------------

clear all; close all; clc; 



%--------------------------------------------------------------------------
%% BEGIN MODEL
%--------------------------------------------------------------------------

%% SPECIFY SETTINGS
%--------------------------------------------------------------------------
% Data
dataPath = '<insert data path>';
                                 % Location of the data file
fileName = 'IOData';

% PGNN settings
n_a = 5; n_b = 1; n_k = 2;      % Order of the dynamics
networkSize         = [16];     % Specify the number of neurons per hidden layer, e.g., [16, 16] gives two 16-neuron layers
typeOfTransform     = 1;        % Type of regressor as inputted in the physics-based transform.

% Training settings
lambda              = logspace(-5, 1, 20);
lambda_phy          = 1;        % L2 regularize the deviation of the physical parameters w.r.t. physical parameters obtained using physical model, insert a scalar or a column with dimension of physical parameters
lambda_NN           = 0.1;     % L2 regularize the NN parameters, input a scalar or a column with dimension of the NN params
gamma_ZN            = 0;        % PINNs regularization
gamma_ZE            = 0;        % PINNs regularization for evaluated over ZE
reg_params = {lambda_phy, lambda_NN, gamma_ZN, gamma_ZE};

% Specialized settings (type of training, data processing)
partValData         = 0.3;      % Part of the data that is used for validation
downSampling        = 1;        % Factor to downsample, i.e., keep 1 point every <downSampling> points
numberOfTrainings   = 20;        % Number of independent trainings to be performed for each configuration, after which the best performing one is chosen
useInitialization   = 1;        % If == 1 -> update initial (PG)NN parameters using optimized initialization, if == 0 -> do not
randomInitAlways    = 0;        % If == 1 -> initialize training randomly, if == 0 -> initialize using thetahat of the previously trained PGNN (when multiple values for lambda are chosen)

% To be implemented
typeOfInverse       = 0;        % 0: Direct inverse ARX, 1: Direct forward ARX; 2: Direct inverse OE; 3: Direct forward OE; etc.





%% 1. LOAD AND PREPARE DATA SET
%--------------------------------------------------------------------------
fprintf('1/4. Load and prepare data.\n');
addpath(dataPath); load(fileName); rmpath(dataPath);    % Load the data
Ts = (t(end)-t(1))/(length(t)-1);       % Compute sampling time

% Construct phi and output
[phi, output, phi_val, output_val, phi_E] = generatePhiOutput(u, y, [n_a,n_b,n_k], typeOfInverse, partValData, reg_params, Ts);
% Downsample the amount of training data to allow for quick training
phi = phi(:, 1:downSampling:size(phi,2)); output = output(:, 1:downSampling:size(output,2));

%% 2. PERFORM PHYSICS-BASED ID AND INITIALIZE NN
%--------------------------------------------------------------------------
fprintf('2/4. Perform physics-based identification.\n');
% Perform identification of the physics-based parameters
theta_PGstar = identifyPhysicsBasedParameters(phi, output, Ts, typeOfTransform);
save(['theta_PG_ToT',num2str(typeOfTransform)], 'theta_PGstar');



%% 3. PERFORM PGNN TRAININGS
%--------------------------------------------------------------------------
fprintf('3/4. Perform PGNN training:\n');
for ii = 1:1:size(lambda, 2)
    fprintf(['  ', num2str(ii), '/', num2str(size(lambda,2)), ' regularizations:\n']);
    if ((ii == 1) || (randomInitAlways == 1))   % If training is performed for the first time, do regular training
        for jj = 1:1:numberOfTrainings
            fprintf(['      ', num2str(jj), '/', num2str(numberOfTrainings), ' trainings: ']);
            % Initialize PGNN
            [theta_0, n_params] = PGNN_Initialize(output, phi, phi_E, Ts, typeOfTransform, theta_PGstar, networkSize, lambda(ii), reg_params, useInitialization);     
            fprintf([' initialized X']);
            % Train PGNN
            [theta(:,jj), resNorm(jj), resNormVal(jj)] = PGNN_Optimization(output, phi, output_val, phi_val, phi_E, Ts, typeOfTransform, theta_0, theta_PGstar, networkSize, n_params, lambda(ii), reg_params);
            fprintf([', trained X.\n']);
        end
    else% Initialize according to parameters obtained for previous value of regularization parameter lambda
        % Observe: we do not use random initialization and the solver is
        % deterministic, hence we only require one training.
        fprintf(['      1/1: use thetahat of previous lambda for initialization.\n']);
        clear theta resNorm resNormVal;     % Clear these variables, to ensure correct dimensions
        % Initialize theta_0 using thetahat of previous lambda
        %load(['PGNN_ARX_', num2str(networkSize), '_Phi', num2str(typeOfTransform), '_lambda', num2str(ii-1)]);
        load(['PGNN_ARX_', num2str(ii-1)]);
        theta_0 = thetahat;
        % Train PGNN
        [theta, resNorm, resNormVal] = PGNN_Optimization(output, phi, output_val, phi_val, phi_E, Ts, typeOfTransform, theta_0, theta_PGstar, networkSize, n_params, lambda(ii), reg_params);
    end
    % Save the trained PGNN
    [~, best] = min(resNorm); thetahat = theta(:, best);
    %save(['PGNN_ARX_', num2str(networkSize), '_Phi', num2str(typeOfTransform), '_lambda', num2str(ii)], 'thetahat', 'networkSize', 'n_params', 'Ts');
    save(['PGNN_ARX_', num2str(ii)], 'thetahat', 'networkSize', 'n_params', 'Ts')
end



%% 4. VISUALIZE AND VALIDATE RESULTS
%--------------------------------------------------------------------------
fprintf('4/4. Visualize and validate results.\n');
visualize_Results;








