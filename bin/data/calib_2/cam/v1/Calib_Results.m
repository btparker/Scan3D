% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 2862.523783551791000 ; 2866.639916495768400 ];

%-- Principal point:
cc = [ 738.574817096328730 ; 568.030448401409440 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.085412027481290 ; 0.594874534973045 ; -0.006960949935939 ; -0.009032259513763 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 20.657459400498542 ; 19.766606918901214 ];

%-- Principal point uncertainty:
cc_error = [ 17.302147286628418 ; 15.201535210010304 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.039471893400627 ; 0.457301085194830 ; 0.001857717916674 ; 0.002159920218494 ; 0.000000000000000 ];

%-- Image size:
nx = 1600;
ny = 1200;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 15;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -3.090264e+000 ; 3.142089e-002 ; -3.763786e-001 ];
Tc_1  = [ -1.464776e+002 ; 1.239743e+002 ; 7.720254e+002 ];
omc_error_1 = [ 7.546137e-003 ; 1.544765e-003 ; 1.175307e-002 ];
Tc_error_1  = [ 4.725837e+000 ; 4.119636e+000 ; 5.664300e+000 ];

%-- Image #2:
omc_2 = [ -3.020438e+000 ; -1.963830e-002 ; 1.344072e-001 ];
Tc_2  = [ -1.551658e+002 ; 1.226798e+002 ; 7.845862e+002 ];
omc_error_2 = [ 7.341355e-003 ; 1.233078e-003 ; 1.212295e-002 ];
Tc_error_2  = [ 4.771292e+000 ; 4.168524e+000 ; 5.643736e+000 ];

%-- Image #3:
omc_3 = [ -2.732279e+000 ; 5.671061e-003 ; 3.520433e-002 ];
Tc_3  = [ -1.187338e+002 ; 1.300883e+002 ; 7.591074e+002 ];
omc_error_3 = [ 5.993879e-003 ; 2.100958e-003 ; 9.748659e-003 ];
Tc_error_3  = [ 4.629679e+000 ; 4.021092e+000 ; 5.190367e+000 ];

%-- Image #4:
omc_4 = [ -2.898881e+000 ; 4.262908e-002 ; -8.450076e-001 ];
Tc_4  = [ -9.759249e+001 ; 1.316482e+002 ; 7.165993e+002 ];
omc_error_4 = [ 6.332192e-003 ; 2.691397e-003 ; 9.736067e-003 ];
Tc_error_4  = [ 4.381428e+000 ; 3.809563e+000 ; 5.354511e+000 ];

%-- Image #5:
omc_5 = [ -2.708611e+000 ; -4.071417e-002 ; -7.940027e-001 ];
Tc_5  = [ -9.120247e+001 ; 1.343717e+002 ; 7.067369e+002 ];
omc_error_5 = [ 5.993728e-003 ; 3.111541e-003 ; 9.067157e-003 ];
Tc_error_5  = [ 4.310719e+000 ; 3.752242e+000 ; 5.175979e+000 ];

%-- Image #6:
omc_6 = [ 3.018692e+000 ; -3.569945e-002 ; 3.691173e-001 ];
Tc_6  = [ -9.681463e+001 ; 1.272764e+002 ; 7.377114e+002 ];
omc_error_6 = [ 7.605864e-003 ; 1.724510e-003 ; 1.165253e-002 ];
Tc_error_6  = [ 4.514424e+000 ; 3.918284e+000 ; 5.498741e+000 ];

%-- Image #7:
omc_7 = [ -3.103245e+000 ; -8.477092e-003 ; 2.223814e-001 ];
Tc_7  = [ -1.396860e+002 ; 1.291014e+002 ; 7.530836e+002 ];
omc_error_7 = [ 7.213455e-003 ; 1.197837e-003 ; 1.214311e-002 ];
Tc_error_7  = [ 4.586091e+000 ; 4.003784e+000 ; 5.413688e+000 ];

%-- Image #8:
omc_8 = [ -2.762374e+000 ; 3.376030e-003 ; 1.646336e-001 ];
Tc_8  = [ -1.041699e+002 ; 1.342723e+002 ; 7.250275e+002 ];
omc_error_8 = [ 5.943565e-003 ; 1.998813e-003 ; 9.768884e-003 ];
Tc_error_8  = [ 4.433079e+000 ; 3.832896e+000 ; 4.872836e+000 ];

%-- Image #9:
omc_9 = [ -2.967078e+000 ; -5.973353e-002 ; 5.914603e-001 ];
Tc_9  = [ -1.325755e+002 ; 1.242774e+002 ; 7.765771e+002 ];
omc_error_9 = [ 6.455427e-003 ; 1.938261e-003 ; 1.032063e-002 ];
Tc_error_9  = [ 4.745599e+000 ; 4.124579e+000 ; 5.119715e+000 ];

%-- Image #10:
omc_10 = [ 3.042358e+000 ; -6.219717e-002 ; 3.671359e-001 ];
Tc_10  = [ -1.345503e+002 ; 1.296295e+002 ; 7.363340e+002 ];
omc_error_10 = [ 7.338126e-003 ; 1.569199e-003 ; 1.117275e-002 ];
Tc_error_10  = [ 4.518956e+000 ; 3.926835e+000 ; 5.491885e+000 ];

%-- Image #11:
omc_11 = [ -2.734162e+000 ; -3.042444e-002 ; -3.546225e-001 ];
Tc_11  = [ -8.997442e+001 ; 1.343192e+002 ; 7.177157e+002 ];
omc_error_11 = [ 6.032020e-003 ; 2.481223e-003 ; 9.560492e-003 ];
Tc_error_11  = [ 4.376598e+000 ; 3.803473e+000 ; 5.032737e+000 ];

%-- Image #12:
omc_12 = [ -3.026001e+000 ; 6.755527e-003 ; 3.588649e-001 ];
Tc_12  = [ -1.398468e+002 ; 1.285945e+002 ; 7.695154e+002 ];
omc_error_12 = [ 6.471064e-003 ; 1.437865e-003 ; 1.113296e-002 ];
Tc_error_12  = [ 4.689550e+000 ; 4.081392e+000 ; 5.314280e+000 ];

%-- Image #13:
omc_13 = [ -2.836850e+000 ; 5.404441e-002 ; -9.987824e-001 ];
Tc_13  = [ -8.251058e+001 ; 1.363863e+002 ; 6.876482e+002 ];
omc_error_13 = [ 6.191303e-003 ; 3.073587e-003 ; 9.373089e-003 ];
Tc_error_13  = [ 4.206112e+000 ; 3.652678e+000 ; 5.194625e+000 ];

%-- Image #14:
omc_14 = [ -2.810426e+000 ; -5.254707e-002 ; -4.039042e-001 ];
Tc_14  = [ -9.322883e+001 ; 1.308367e+002 ; 7.103435e+002 ];
omc_error_14 = [ 6.375101e-003 ; 2.510741e-003 ; 1.011770e-002 ];
Tc_error_14  = [ 4.333573e+000 ; 3.763805e+000 ; 5.116970e+000 ];

%-- Image #15:
omc_15 = [ 3.016216e+000 ; -1.397984e-002 ; 3.696438e-001 ];
Tc_15  = [ -1.008966e+002 ; 1.251874e+002 ; 7.382499e+002 ];
omc_error_15 = [ 6.741141e-003 ; 1.589756e-003 ; 1.053778e-002 ];
Tc_error_15  = [ 4.510813e+000 ; 3.920757e+000 ; 5.468299e+000 ];

