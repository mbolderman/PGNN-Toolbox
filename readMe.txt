PGNN TOOLBOX.

DISCLAIMER:
Usage of the PGNN toolbox is free, under the condition that satisfactory credit is given by citing the paper:
[1] M. Bolderman, M. Lazar, H. Butler, A MATLAB toolbox for training and implementing physics-guided neural network-based feedforward 
	controllers, IFAC World Congress (2022). 

This work is part of the research programme with project number 17973, which is (partly) financed by the Dutch Research Council (NWO). 

Control Systems Group, Electrical Engineering, Eindhoven University of Technology. 
Groene Loper 19, 5612 AP Eindhoven, The Netherlands. 




SUMMARY:
The toolbox systematically implements, trains and validates PGNN-based feedforward controllers. 
More information on the theory, and implementation is presented in the accompnaying paper [1]. 


TOOLBOX DEPENDENCIES
1. The toolbox uses Matlab's "lsqnonlin()" optimization, part of the "Optimization Toolbox". 


RUNNING THE TOOLBOX:
1. Open the file "Main_PGNN.m";
2. Specify the settings:
	2.a. Insert the dataPath and fileName of the input-output data set.
	2.b. Insert the desired settings for the PGNN to be trained, i.e., dimensions and regularization parameters.
3. Run "Main_PGNN.m". The toolbox returns:
	3.a. Figures of the generated feedforward signals when the PGNN is evaluated on the references saved in "Reference1.mat" and 
	     "Reference2.mat", the L-curve, and the value of the cost function.
	3.b. A file that contains the identified parameters and network dimensions to compute the PGNN feedforward.
	

APPLICATION OF THE TOOLBOX TO A NEW PROBLEM:
1. Choosing a different data set:
	1.a. Save a data set that contains at least u = [u(0), ..., u(N-1)], y = [y(0), ..., y(N-1)], and t = [t(0), ..., t(N-1)];
	1.b. Adjust "dataPath" and "fileName" accordingly in "Main_PGNN.m". 
2. Adjusting the NN input transformation, NN activation function, and physical model:
	2.a. Open PGNN_PGT.m and insert the desired transformation either in a yet existing option for "typeOfTransform", or create a new 
 	     option by imposing another "elseif" condition. Ensure that the value for "typeOfTransform" in "Main_PGNN.m" is correct;
	2.b. Open "identifyPhysicsBasedParameters.m" and identify as desired, e.g., when it is desired to fix certain physical parameters;
	2.c. When a NN is trained, i.e., PGNN without physical model, adjust the physical model used for regularization in 
	     "PG_ModelOutput.m" if gamma_ZN and/or gamma_ZE > 0;
	2.d. Open "NN_ActivationFunction.m" and insert the desired activation function. 
3. Evaluating the trained PGNNs on different references:
	3.a. Save r = [r(0), ..., r(N-1)] in <referenceName>, and load <referenceName> in "visualize_Results.m". 
4. Real-time implementation in Simulink environment:
	4.a. Save "NN_ActivationFunction.m", "NN_Output.m", "PGNN_Output.m", "PGNN_PGT", and the PGNN file, e.g., 
	     "PGNN_ARX_16_Phi1_lambda1.mat" in a folder on the host PC, and add the folder to the path;
	4.b. Put a "Matlab function" block in the Simulink environment and insert the required inputs;
	4.c. Put the following code to compute the PGNN feedforward:
		x = coder.load("<PGNN_File>");
		networkSize = x.networkSize; n_params = x.n_params; thetahat = x.thetahat;
		phi_ff = [r(k+n_k+1); ...; r(k+n_k-n_a); u_ff(k-1); ...; u_ff(k-n_b+1)];	% <- put here the correct variable names
		u_ff = PGNN_Output(phi_ff, Ts, typeOfTransform, thetahat, networkSize, n_params); 	
	4.d. Some versions of Simulink experience trouble when computing the NN output using the recursive algorithm. A quick fix is to 
	     hardcode the recursion for the number of hidden layers in "NN_Output.m". 


THEORETICAL BACKGROUND:
Theory of the PGNN framework, regularization terms, and optimized initialization, inversion methods, and stability validation has been published in:
[1] M. Bolderman, M. Lazar, H. Butler, A MATLAB toolbox for training and implementing physics-guided neural network-based feedforward 
	controllers, IFAC World Congress (2022). 
[2] M. Bolderman, M. Lazar, H. Butler, Physics-guided neural networks for inversion-based feedforward control applied to linear motors, 
	IEEE Conference on Control Technology and Applications (2021) 1115-1120.
[3] M. Bolderman, M. Lazar, H. Butler, On feedforward control using physics-guided neural networks: Training cost regularization and 
	optimized initialization, European Control Conference (2022) 1403-1408. 
[4] M. Bolderman, D. Fan, M. Lazar, H. Butler, Generalized feedforward control using physics-informed neural networks, 
	IFAC-PapersOnline 55 (2022) 148-153.
[5] M. Bolderman, M. lazar, H. Butler, Physics-guided neural networks for feedforward control: From consistent identification to feedforward
	controller design, IEEE Conference on Decision and Control (2022). 
[6] M. Bolderman, M. Lazar, H. Butler, Generalized feedforward control design using physics-guided neural networks, in preparation (2022).




