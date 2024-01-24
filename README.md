Code Access and Overview

The supplementary materials include Matlab code for implementing the simulations and analysis presented in this paper. The following scripts are included:

Main Script
- Paper1_supply_2.m: Main Matlab script that runs the simulations. Executes the optimization using the fmincon solver and calls the SLIP_optim function.

Functions  
- SLIP_optim.m: Defines the subject function for nonlinear optimization with fmincon. 
- SLIP_eom.m: Defines system of differential equations (equations of motion) and it is executed through ODE45 solver.

Usage Guide
To run the simulations, first open Paper1_supply_2.m in Matlab. This will execute the fmincon optimization, utilizing SLIP_optim for the objective function definition. 
SLIP_optim leverages SLIP_eom to numerically integrate the equations of motion for the system. Please refer to the comments in each file for further implementation details 
and parameter descriptions. When specific model parameters are selected, this code is executed to determine an appropriate initial value for state variables.  In this particular script,
the optimization aims to find the value of stiffness (k) based on given touchdown angle (th_td) and initial compression (Del_ltd) parameters, resulting in the determination of locomotion speed. 
The provided code serves as an illustration of deriving a jogging gait with th_td = 0.2 and Del_ltd = 0.06, yielding a periodic gait at k = 6619. 
Modifications are made to the model parameters for each distinct outcome depicted in the main manuscript.

N.B.: Let me know if you would like me to elaborate on or clarify any part of this organization and description! 
