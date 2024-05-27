%%  Three-dimensional muscle-driven predictive simulations of human gaits
%
% Author: Antoine Falisse
% Date: 1/7/2019
%
% Edited by Gilmar Fernandes dos Santos on 03.10.2023
clear all;
clc
close all;

%% User inputs
% This script can be run to solve the optimal control problems but also to
% analyze and process the results. The user does not need to re-run the
% optimal control problems to analyze the results. Therefore, the user can
% select some settings beforehand through the variable num_set. For
% example, when num_set(1) is set to 0, the script will not run the
% optimal control problem. Here is a brief description of num_set:
% num_set(1): set to 1 to solve problem
% num_set(2): set to 1 to analyze results
% num_set(3): set to 1 to load results
% num_set(4): set to 1 to save results
% num_set(5): set to 1 to visualize guess-bounds 
% num_set(6): set to 1 to write .mot file
% num_set(7): set to 1 to decompose cost
% Note that you should re-run the simulations to write out the .mot files
% and visualize the results in the OpenSim GUI.

num_set = [1,1,0,1,0,1,0]; % This configuration solves the problem
% num_set = [0,1,1,1,0,1,1]; % This configuration analyzes the results

% The variable settings in the following section will set some parameters 
% of the optimal control problems. Through the variable idx_ww, the user  
% can select which row of parameters will be used.
idx_ww = [829]; % Index row in matrix settings (1:198) idx_ww = [x;y];

%% Settings
import casadi.*
subject = 'subjectAB';

solveProblem    = num_set(1); % set to 1 to solve problem
analyseResults  = num_set(2); % set to 1 to analyze results
loadResults     = num_set(3); % set to 1 to load results
saveResults     = num_set(4); % set to 1 to save sens. results
checkBoundsIG   = num_set(5); % set to 1 to visualize guess-bounds 
writeIKmotion   = num_set(6); % set to 1 to write .mot file
decomposeCost   = num_set(7); % set to 1 to decompose cost

p_new = mfilename('fullpath');
[folderscript,~,~] = fileparts(p_new);
cd(folderscript)

% Paths
pathmain3 = pwd;
[pathmain2,~,~] = fileparts(pathmain3);
[path_2,~,~] = fileparts(pathmain2);
pathSettings = [path_2,'/Settings'];
addpath(genpath(pathSettings));
Settings_PredSim



%% Select settings
for www = 1:length(idx_ww)

%% Set parameters based on settings
ww = idx_ww(www);
% Variable parameters
v_tgt       = settings(ww,1);    % average speed
tol_ipopt   = settings(ww,2);    % tolerance (means 1e-(tol_ipopt))
N           = settings(ww,3);    % number of mesh intervals
W.E         = settings(ww,4);    % weight metabolic energy rate
W.Ak        = settings(ww,5);    % weight joint accelerations
W.ArmE      = settings(ww,6);    % weight arm excitations
W.passMom   = settings(ww,7);    % weight passive torques
W.A         = settings(ww,8);    % weight muscle activations
exp_E       = settings(ww,9);    % power metabolic energy
IGsel       = settings(ww,10);   % initial guess identifier
cm          = settings(ww,11);   % contact model identifier
IGm         = settings(ww,12);   % initial guess mode identifier
IGcase      = settings(ww,13);   % initial guess case identifier
h_weak      = settings(ww,14);   % weakness hip actuators
vMax_s      = settings(ww,15);   % maximal contraction velocity identifier
pf_weak     = settings(ww,16);   % weakness ankle plantaflexors
mE          = settings(ww,17);   % metabolic energy model identifier
coCont      = settings(ww,18);   % co-contraction identifier
% Fixed parameter
W.u = 0.001;
% Passive mtp
k_pass_mtp = 25;
d_pass_mtp = 0.4;
% The filename used to save the results depends on the settings 
v_tgt_id = round(v_tgt,2);
savename = ['_c',num2str(ww),'_v',num2str(v_tgt_id*100),...
    '_T',num2str(tol_ipopt),'_N',num2str(N),'_E',num2str(W.E),...
    '_Ak',num2str(W.Ak),'_AE',num2str(W.ArmE),'_P',num2str(W.passMom),...
    '_A',num2str(W.A),'_eE',num2str(exp_E),'_G',num2str(IGsel),...
    '_M',num2str(cm),'_Gm',num2str(IGm),...
    '_W',num2str(h_weak),'_vM',num2str(vMax_s),...
    '_pW',num2str(pf_weak),'_mE',num2str(mE),'_cc',num2str(coCont)];

% In some cases, the inital guess depends on results from other simulations
if IGm == 3 || IGm == 4    
ww_ig          = IGcase;               % Case identifier used as IG       
v_tgt_ig       = settings(ww_ig,1);    % average speed
tol_ipopt_ig   = settings(ww_ig,2);    % tolerance (means 1e-(tol_ipopt))
N_ig           = settings(ww_ig,3);    % number of mesh intervals
W.E_ig         = settings(ww_ig,4);    % weight metabolic energy
W.Ak_ig        = settings(ww_ig,5);    % weight joint accelerations
W.ArmE_ig      = settings(ww_ig,6);    % weight arm excitations
W.passMom_ig   = settings(ww_ig,7);    % weight passive torques
W.A_ig         = settings(ww_ig,8);    % weight muscle activations
exp_E_ig       = settings(ww_ig,9);    % power metabolic energy
IGsel_ig       = settings(ww_ig,10);   % initial guess identifier
cm_ig          = settings(ww_ig,11);   % contact model identifier
IGm_ig         = settings(ww_ig,12);   % initial guess mode identifier
% There is no need for index 13, since it is not in savename
h_weak_ig      = settings(ww_ig,14);   % weakness hip actuators
vMax_s_ig      = settings(ww_ig,15);   % maximal contraction velocity id
pf_weak_ig     = settings(ww_ig,16);   % weakness ankle plantarflexors
ME_ig          = settings(ww_ig,17);   % metabolic energy model identifier
coCont_ig      = settings(ww_ig,18);   % co-contraction identifier

% The filename used to load the results depends on the settings 
v_tgt_id_ig = round(v_tgt_ig,2);
savename_ig = ['_c',num2str(ww_ig),'_v',num2str(v_tgt_id_ig*100),...
    '_T',num2str(tol_ipopt_ig),'_N',num2str(N_ig),'_E',num2str(W.E_ig),...
    '_Ak',num2str(W.Ak_ig),'_AE',num2str(W.ArmE_ig),...
    '_P',num2str(W.passMom_ig),'_A',num2str(W.A_ig),...
    '_eE',num2str(exp_E_ig),'_G',num2str(IGsel_ig),'_M',num2str(cm_ig),...
    '_Gm',num2str(IGm_ig),'_W',num2str(h_weak_ig),'_vM',num2str(vMax_s_ig),...
    '_pW',num2str(pf_weak_ig),'_mE',num2str(ME_ig),'_cc',num2str(coCont_ig)];
end

%% Load external functions
% The external function performs inverse dynamics through the
% OpenSim/Simbody C++ API. This external function is compiled as a dll from
% which we create a Function instance using CasADi in MATLAB. More details
% about the external function can be found in the documentation.
pathmain = pwd;
% We use different external functions, since we also want to access some 
% parameters of the model in a post-processing phase.
[pathRepo4,~,~] = fileparts(pathmain);
[pathRepo,~,~] = fileparts(pathRepo4);
pathExternalFunctions = [pathRepo,'/ExternalFunctions/',subject];
% Loading external functions. 
cd(pathExternalFunctions);
setup.derivatives =  'AD'; % Algorithmic differentiation
if ispc    
    switch setup.derivatives
        case {'AD'}   
            if cm == 1
                F = external('F','PredSim_AB1.dll');   
                if analyseResults
                    F1 = external('F','PredSim_AB1_pp.dll');
                end
            elseif cm == 2
                F = external('F','PredSim_AB1_SSCM.dll');   
                if analyseResults
                    F1 = external('F','PredSim_AB1_SSCM_pp.dll');
                end
            else
                name_Fext = ['F = external(''F'',''PredSim_AB1_SSCM',num2str(cm),'.dll'');'];
                eval(name_Fext);
                if analyseResults
                    name_Fext1 = ['F1 = external(''F'',''PredSim_AB1_SSCM',num2str(cm),'_pp.dll'');'];
                    eval(name_Fext1);
                end
            end
    end
elseif ismac
    switch setup.derivatives
        case {'AD'}   
            if cm == 1
                F = external('F','PredSim.dylib');   
                if analyseResults
                    F1 = external('F','PredSim_pp.dylib');
                end
            elseif cm == 2
                F = external('F','PredSim_SSCM.dylib');   
                if analyseResults
                    F1 = external('F','PredSim_SSCM_pp.dylib');
                end
            end
    end
else
    disp('Platform not supported')
end
cd(pathmain);
% This is an example of how to call an external function with some
% numerical values.
% vec1 = -ones(87,1);
% res1 = full(F(vec1));
% res2 = full(F1(vec1));

%% Indices external function
% Indices of the elements in the external functions
% External function: F
% First, joint torques. 
jointi.pelvis.tilt  = 1; 
jointi.pelvis.list  = 2; 
jointi.pelvis.rot   = 3; 
jointi.pelvis.tx    = 4;
jointi.pelvis.ty    = 5;
jointi.pelvis.tz    = 6;
jointi.hip_flex.l   = 7;
jointi.hip_add.l    = 8;
jointi.hip_rot.l    = 9;
jointi.hip_flex.r   = 10;
jointi.hip_add.r    = 11;
jointi.hip_rot.r    = 12;
jointi.knee.l       = 13;
jointi.knee.r       = 14;
jointi.ankle.l      = 15;
jointi.ankle.r      = 16;
jointi.subt.l       = 17;
jointi.subt.r       = 18;
jointi.mtp.l       = 19;
jointi.mtp.r       = 20;
jointi.trunk.ext    = 19+2;
jointi.trunk.ben    = 20+2;
jointi.trunk.rot    = 21+2;

% Vectors of indices for later use
% residualsi          = jointi.pelvis.tilt:jointi.elb.r; % all 
residualsi          = jointi.pelvis.tilt:jointi.trunk.rot; % all 
ground_pelvisi      = jointi.pelvis.tilt:jointi.pelvis.tz; % ground-pelvis
trunki              = jointi.trunk.ext:jointi.trunk.rot; % trunk
% armsi               = jointi.sh_flex.l:jointi.elb.r; % arms
residuals_noarmsi   = jointi.pelvis.tilt:jointi.trunk.rot; % all but arms
% Number of degrees of freedom for later use
nq.all      = length(residualsi); % all 
nq.abs      = length(ground_pelvisi); % ground-pelvis
nq.trunk    = length(trunki); % trunk
nq.leg      = 17; % #joints needed for polynomials
% Second, origins bodies. %GFdS: Joint origins on PredSim.cpp (remember that Matlab starts at 1 and CPP at 0) 
% Calcaneus
calcOr.r    = 22+2:23+2;
calcOr.l    = 24+2:25+2;
calcOr.all  = [calcOr.r,calcOr.l];
NcalcOr     = length(calcOr.all);
% Femurs
femurOr.r   = 26+2:27+2;
femurOr.l   = 28+2:29+2;
femurOr.all = [femurOr.r,femurOr.l];
NfemurOr    = length(femurOr.all);
tibiaOr.r   = 30+2:31+2;
tibiaOr.l   = 32+2:33+2;
tibiaOr.all = [tibiaOr.r,tibiaOr.l];
NtibiaOr    = length(tibiaOr.all);
% External function: F1 (post-processing purpose only)
% Ground reaction forces (GRFs) 
GRFi.r      = 22+2:24+2;
GRFi.l      = 25+2:27+2;
GRFi.all    = [GRFi.r,GRFi.l];
NGRF        = length(GRFi.all);
% Origins calcaneus (3D)
calcOrall.r     = 28+2:30+2;
calcOrall.l     = 31+2:33+2;
calcOrall.all   = [calcOrall.r,calcOrall.l];
NcalcOrall      = length(calcOrall.all);

%% Model info
body_mass = 72.1; 
body_weight = body_mass*9.81;

%% Collocation scheme
% We use a pseudospectral direct collocation method, i.e. we use Lagrange
% polynomials to approximate the state derivatives at the collocation
% points in each mesh interval. We use d=3 collocation points per mesh
% interval and Radau collocation points. 
pathCollocationScheme = [pathRepo,'/CollocationScheme'];
addpath(genpath(pathCollocationScheme));
d = 3; % degree of interpolating polynomial
method = 'radau'; % collocation method
[tau_root,C,D,B] = CollocationScheme(d,method);

%% Muscle-tendon parameters 
% Muscles from both legs and from the back
muscleNames = {'glut_med1_l','glut_med2_l','glut_med3_l',...
        'glut_min1_l','glut_min2_l','glut_min3_l','semimem_l',...
        'semiten_l','bifemlh_l','bifemsh_l','sar_l','add_long_l',...
        'add_brev_l','add_mag1_l','add_mag2_l','add_mag3_l','tfl_l',...
        'pect_l','grac_l','glut_max1_l','glut_max2_l','glut_max3_l',...
        'iliacus_l','psoas_l','quad_fem_l','gem_l','peri_l',...
        'rect_fem_l','vas_med_l','vas_int_l','vas_lat_l','med_gas_l',...
        'lat_gas_l','soleus_l','tib_post_l','flex_dig_l','flex_hal_l',...
        'tib_ant_l','per_brev_l','per_long_l','per_tert_l','ext_dig_l',...
        'ext_hal_l','ercspn_l','intobl_l','extobl_l',...
        'glut_med1_r','glut_med2_r','glut_med3_r',...
        'glut_min1_r','glut_min2_r','glut_min3_r','semimem_r',...
        'semiten_r','bifemlh_r','bifemsh_r','sar_r','add_long_r',...
        'add_brev_r','add_mag1_r','add_mag2_r','add_mag3_r','tfl_r',...
        'pect_r','grac_r','glut_max1_r','glut_max2_r','glut_max3_r',......
        'iliacus_r','psoas_r','quad_fem_r','gem_r','peri_r',...
        'rect_fem_r','vas_med_r','vas_int_r','vas_lat_r','med_gas_r',...
        'lat_gas_r','soleus_r','tib_post_r','flex_dig_r','flex_hal_r',...
        'tib_ant_r','per_brev_r','per_long_r','per_tert_r','ext_dig_r',...
        'ext_hal_r','ercspn_r','intobl_r','extobl_r'};
% Muscle indices for later use
pathmusclemodel = [pathRepo,'/MuscleModel'];
addpath(genpath(pathmusclemodel)); 
% (1:end-3), since we do not want to count twice the back muscles
musi = MuscleIndices_Gil3(muscleNames);
% Total number of muscles
NMuscle = length(muscleNames);
% Muscle-tendon parameters. Row 1: maximal isometric forces; Row 2: optimal
% fiber lengths; Row 3: tendon slack lengths; Row 4: optimal pennation 
% angles; Row 5: maximal contraction velocities
load([pathmusclemodel,'/MTparameters_',subject,'.mat']);
MTparameters_m = [MTparameters];
clear MTparameters
% Indices of the muscles actuating the different joints for later use
pathpolynomial = [pathRepo,'/Polynomials/',subject];
addpath(genpath(pathpolynomial));
tl = load([pathpolynomial,'/muscle_spanning_joint_INFO_',subject,'.mat']);
[~,mai] = MomentArmIndices_Gil4(muscleNames,...
    tl.muscle_spanning_joint_INFO);

% By default, the tendon stiffness is 35 and the shift is 0.
aTendon = 35*ones(NMuscle,1);
shift = zeros(NMuscle,1);

% Adjust the maximal isometric force of the hip actuators if needed.
if h_weak ~= 0 
    MTparameters_m(1,[mai(1).mus.l,mai(1).mus.r]) = ...
        MTparameters_m(1,[mai(1).mus.l,mai(1).mus.r])-...
        h_weak/100*MTparameters_m(1,[mai(1).mus.l,mai(1).mus.r]);
end 

% Adjust the maximal isometric force of the ankle plantarflexors if needed.
if pf_weak ~= 0
    idx_FD = find(strcmp(muscleNames,'flex_dig_r'));
    idx_FH = find(strcmp(muscleNames,'flex_hal_r'));
    idx_GL = find(strcmp(muscleNames,'lat_gas_r'));
    idx_GM = find(strcmp(muscleNames,'med_gas_r'));
    idx_PB = find(strcmp(muscleNames,'per_brev_r'));
    idx_PL = find(strcmp(muscleNames,'per_long_r'));
    idx_SO = find(strcmp(muscleNames,'soleus_r'));
    idx_TP = find(strcmp(muscleNames,'tib_post_r'));    
    idx_pf = [idx_FD,idx_FH,idx_GL,idx_GM,idx_PB,idx_PL,idx_SO,idx_TP];
    idx_pf_all = [idx_pf,idx_pf+NMuscle/2];        
    MTparameters_m(1,idx_pf_all) = MTparameters_m(1,idx_pf_all)-...
        pf_weak/100*MTparameters_m(1,idx_pf_all);
end
all_weakness = 'none';
% Adjust the maximum contraction velocities if needed.
if vMax_s == 1
    % Maximum contraction velocities * 2
    MTparameters_m(end,:) = MTparameters_m(end,:).*2;
elseif vMax_s == 2
    % Maximum contraction velocities * 1.5
    MTparameters_m(end,:) = MTparameters_m(end,:).*1.5;
end

%% Metabolic energy model parameters
% We extract the specific tensions and slow twitch rations.
pathMetabolicEnergy = [pathRepo,'/MetabolicEnergy'];
addpath(genpath(pathMetabolicEnergy));
tension = getSpecificTensions_Gil2(muscleNames); 
tensions = [tension];
pctst = getSlowTwitchRatios_Gil2(muscleNames); 
pctsts = [pctst];

%% CasADi functions
% We create several CasADi functions for later use
pathCasADiFunctions = [pathRepo,'/CasADiFunctions'];
addpath(genpath(pathCasADiFunctions));
% We load some variables for the polynomial approximations
load([pathpolynomial,'/muscle_spanning_joint_INFO_',subject,'.mat']);
load([pathpolynomial,'/MuscleInfo_',subject,'.mat']);

musi_pol = [musi];
NMuscle_pol = NMuscle;
CasADiFunctions_all_Gil19

%% Passive joint torques
% We extract the parameters for the passive torques of the lower limbs and
% the trunk
pathPassiveMoments = [pathRepo,'/PassiveMoments'];
addpath(genpath(pathPassiveMoments));
PassiveMomentsData_Gil35

%% Experimental data
% We extract experimental data to set bounds and initial guesses if needed
pathData = [pathRepo,'/OpenSimModel/',subject];

joints = {'pelvis_tilt','pelvis_list','pelvis_rotation','pelvis_tx',...
    'pelvis_ty','pelvis_tz','hip_flexion_l','hip_adduction_l',...
    'hip_rotation_l','hip_flexion_r','hip_adduction_r','hip_rotation_r',...
    'knee_angle_l','knee_angle_r','ankle_angle_l','ankle_angle_r',...
    'subtalar_angle_l','subtalar_angle_r','mtp_angle_l','mtp_angle_r','lumbar_extension',...
    'lumbar_bending','lumbar_rotation'};
pathVariousFunctions = [pathRepo,'/VariousFunctions'];
addpath(genpath(pathVariousFunctions));
% Extract joint positions from average walking motion
if IGm == 5
nametrial_walk.IK   = 'IK_C7b_average_walk_SS_mtp';
elseif IGm == 6
nametrial_walk.IK   = 'IK_C1_3b_average_walk_SS_mtp';
elseif IGm == 7
nametrial_walk.IK   = 'IK_average_walking_FGC_mtp_Gil2';
else
motion_walk         = 'walking';
nametrial_walk.id   = ['average_',motion_walk,'_FGC_mtp_Gil1']; 
nametrial_walk.IK   = ['IK_',nametrial_walk.id];
end
pathIK_walk         = [pathData,'/IK/',nametrial_walk.IK,'.mat'];
Qs_walk             = getIK(pathIK_walk,joints);


% Depending on the initial guess mode, we extract further experimental data
if IGm == 2
    % Extract joint positions from average running motion
    motion_run          = 'running';
    nametrial_run.id    = ['average_',motion_run,'_HGC'];  
    nametrial_run.IK    = ['IK_',nametrial_run.id];
    pathIK_run          = [pathData,'/IK/',nametrial_run.IK,'.mat'];
    Qs_run              = getIK(pathIK_run,joints);    
elseif IGm == 3
    % Extract joint positions from existing motion (previous results)
    p = mfilename('fullpath');
    [~,namescript,~] = fileparts(p);
    pathIK = [pathRepo,'/Results/',subject,'/',namescript,'/IK_',namescript,'_c',num2str(IGcase),'.mot'];
    Qs_ig = getIK_Gil2(pathIK,joints);
    % When saving the results, we save a full gait cycle (2*N) so here we
    % only select 1:N to have half a gait cycle
    Qs_ig_sel.allfilt = Qs_ig.allfilt;
    Qs_ig_sel.time = Qs_ig.time;
    Qs_ig_sel.colheaders = Qs_ig.colheaders;
end    

%% Bounds
pathBounds = [pathRepo,'/Bounds'];
addpath(genpath(pathBounds));
[bounds,scaling] = getBounds_all_Gil20_mtp(Qs_walk,NMuscle,nq,jointi,v_tgt);
% Simulate co-contraction by increasing the lower bound on muscle activations
if coCont == 1
    bounds.a.lower = 0.1*ones(1,NMuscle);
elseif coCont == 2
    bounds.a.lower = 0.15*ones(1,NMuscle);
elseif coCont == 3
    bounds.a.lower = 0.2*ones(1,NMuscle);
elseif coCont == 4
    bounds.a.lower = 0.05*ones(1,NMuscle);
end


%% Initial guess
% The initial guess depends on the settings
pathIG = [pathRepo,'/IG'];
addpath(genpath(pathIG));
if IGsel == 1 % Quasi-random initial guess  
    guess = getGuess_QR_Gil3(N,nq,NMuscle,scaling,v_tgt,jointi);
elseif IGsel == 2 % Data-informed initial guess
    if IGm == 1 || IGm == 5 || IGm == 6 || IGm == 7 % Data from average walking motion
        time_IC = [Qs_walk.time(1),Qs_walk.time(end)];
        guess = ...
            getGuess_DI_Gil3(Qs_walk,nq,N,time_IC,NMuscle,jointi,scaling,v_tgt);   
    elseif IGm == 2 % Data from average runing motion    
        time_IC = [Qs_run.time(1),Qs_run.time(end)];
        guess = ...
            getGuess_DI(Qs_run,nq,N,time_IC,NMuscle,jointi,scaling,v_tgt);
    elseif IGm == 3 % Data from selected motion
        time_IC = [Qs_ig_sel.time(1),Qs_ig_sel.time(end)];
        guess = ...
            getGuess_DI_t_Gil2(Qs_ig_sel,nq,N,time_IC,NMuscle,jointi,scaling);    
    end 
end
% If co-contraction, the initial guess of muscles activations is increased
if coCont == 1
    guess.a = 0.15*ones(N,NMuscle);
elseif coCont == 2
    guess.a = 0.20*ones(N,NMuscle);
elseif coCont == 3
    guess.a = 0.25*ones(N,NMuscle);
end
% This allows visualizing the initial guess and the bounds
if checkBoundsIG
    pathPlots = [pathRepo,'/Plots'];
    addpath(genpath(pathPlots));
    plot_BoundsVSInitialGuess_all
end

%% Formulate the NLP
if solveProblem
    % Start with an empty NLP
    w   = {}; % design variables
    w0  = []; % initial guess for design variables
    lbw = []; % lower bounds for design variables
    ubw = []; % upper bounds for design variables
    J   = 0;  % initial value of cost function
    g   = {}; % constraints
    lbg = []; % lower bounds for constraints
    ubg = []; % upper bounds for constraints
    % Define static parameters
    % Final time
    tf              = MX.sym('tf',1);
    w               = [w {tf}];
    lbw             = [lbw; bounds.tf.lower];
    ubw             = [ubw; bounds.tf.upper];
    w0              = [w0;  guess.tf];
    % Define states at first mesh point
    % Muscle activations
    a0              = MX.sym('a0',NMuscle);
    w               = [w {a0}];
    lbw             = [lbw; bounds.a.lower'];
    ubw             = [ubw; bounds.a.upper'];
    w0              = [w0;  guess.a(1,:)'];
    % Muscle-tendon forces
    FTtilde0        = MX.sym('FTtilde0',NMuscle);
    w               = [w {FTtilde0}];
    lbw             = [lbw; bounds.FTtilde.lower'];
    ubw             = [ubw; bounds.FTtilde.upper'];
    w0              = [w0;  guess.FTtilde(1,:)'];
    % Qs and Qdots 
    X0              = MX.sym('X0',2*nq.all);
    w               = [w {X0}];    
    lbw             = [lbw; bounds.QsQdots_0.lower'];
    ubw             = [ubw; bounds.QsQdots_0.upper'];    
    w0              = [w0;  guess.QsQdots(1,:)'];

    % We pre-allocate some of the states so that we can provide an
    % expression for the distance traveled
    for k=0:N
        Xk{k+1,1} = MX.sym(['X_' num2str(k+1)], 2*nq.all);
    end     
    % "Lift" initial conditions
    ak          = a0;
    FTtildek    = FTtilde0;
    Xk{1,1}     = X0;
%     a_ak        = a_a0;     
    % Provide expression for the distance traveled
    pelvis_tx0 = Xk{1,1}(2*jointi.pelvis.tx-1,1).*...
        scaling.QsQdots(2*jointi.pelvis.tx-1); % initial position pelvis_tx     
    pelvis_txf = Xk{N+1,1}(2*jointi.pelvis.tx-1,1).*...
        scaling.QsQdots(2*jointi.pelvis.tx-1); % final position pelvis_tx    
    dist_trav_tot = pelvis_txf-pelvis_tx0;% distance traveled    
    % Time step
    h = tf/N;    
    % Loop over mesh points
    for k=0:N-1
        % Define controls at mesh point (piecewise-constant in interval) 
        % Time derivative of muscle activations (states)
        vAk                 = MX.sym(['vA_' num2str(k)], NMuscle);
        w                   = [w {vAk}];
        lbw                 = [lbw; bounds.vA.lower'];
        ubw                 = [ubw; bounds.vA.upper'];
        w0                  = [w0; guess.vA(k+1,:)'];
        % Time derivative of muscle-tendon forces (states)
        dFTtildek           = MX.sym(['dFTtilde_' num2str(k)], NMuscle);
        w                   = [w {dFTtildek}];
        lbw                 = [lbw; bounds.dFTtilde.lower'];
        ubw                 = [ubw; bounds.dFTtilde.upper'];
        w0                  = [w0; guess.dFTtilde(k+1,:)'];  
        % Time derivative of Qdots (states) 
        Ak                  = MX.sym(['A_' num2str(k)], nq.all);
        w                   = [w {Ak}];
        lbw                 = [lbw; bounds.Qdotdots.lower'];
        ubw                 = [ubw; bounds.Qdotdots.upper'];
        w0                  = [w0; guess.Qdotdots(k+1,:)'];    
        % Define states at collocation points    
        % Muscle activations
        akj = {};
        for j=1:d
            akj{j}  = MX.sym(['	a_' num2str(k) '_' num2str(j)], NMuscle);
            w       = {w{:}, akj{j}};
            lbw     = [lbw; bounds.a.lower'];
            ubw     = [ubw; bounds.a.upper'];
            w0      = [w0;  guess.a(k+1,:)'];
        end   
        % Muscle-tendon forces
        FTtildekj = {};
        for j=1:d
            FTtildekj{j} = ...
                MX.sym(['FTtilde_' num2str(k) '_' num2str(j)], NMuscle);
            w            = {w{:}, FTtildekj{j}};
            lbw          = [lbw; bounds.FTtilde.lower'];
            ubw          = [ubw; bounds.FTtilde.upper'];
            w0           = [w0;  guess.FTtilde(k+1,:)'];
        end
        % Qs and Qdots        
        Xkj = {};
        for j=1:d
            Xkj{j} = MX.sym(['X_' num2str(k) '_' num2str(j)], 2*nq.all);
            w      = {w{:}, Xkj{j}};
            lbw    = [lbw; bounds.QsQdots.lower'];
            ubw    = [ubw; bounds.QsQdots.upper'];
            w0     = [w0;  guess.QsQdots(k+1,:)'];
        end   
        % Unscale variables for later use
        Xk_nsc          = Xk{k+1,1}.*scaling.QsQdots';
        FTtildek_nsc    = FTtildek.*scaling.FTtilde';
        Ak_nsc          = Ak.*scaling.Qdotdots';
        for j=1:d
            Xkj_nsc{j} = Xkj{j}.*scaling.QsQdots';
            FTtildekj_nsc{j} = FTtildekj{j}.*scaling.FTtilde';
        end                
        % Get muscle-tendon lengths, velocities, and moment arms
        % Both legs
        qin_rtl = [Xk_nsc(jointi.hip_flex.r*2-1,1),...
            Xk_nsc(jointi.hip_add.r*2-1,1),...
            Xk_nsc(jointi.hip_rot.r*2-1,1),...
            Xk_nsc(jointi.knee.r*2-1,1),...
            Xk_nsc(jointi.ankle.r*2-1,1),...
            Xk_nsc(jointi.subt.r*2-1,1),...
            Xk_nsc(jointi.mtp.r*2-1,1),...
            Xk_nsc(jointi.trunk.ext*2-1,1),...
            Xk_nsc(jointi.trunk.ben*2-1,1),...
            Xk_nsc(jointi.trunk.rot*2-1,1),...
            Xk_nsc(jointi.hip_flex.l*2-1,1),...
            Xk_nsc(jointi.hip_add.l*2-1,1), ...
            Xk_nsc(jointi.hip_rot.l*2-1,1), ...
            Xk_nsc(jointi.knee.l*2-1,1), ...
            Xk_nsc(jointi.ankle.l*2-1,1),...
            Xk_nsc(jointi.subt.l*2-1,1),...
            Xk_nsc(jointi.mtp.l*2-1,1)];  
        qdotin_rtl = [Xk_nsc(jointi.hip_flex.r*2,1),...
            Xk_nsc(jointi.hip_add.r*2,1),...
            Xk_nsc(jointi.hip_rot.r*2,1),...
            Xk_nsc(jointi.knee.r*2,1),...
            Xk_nsc(jointi.ankle.r*2,1),...
            Xk_nsc(jointi.subt.r*2,1),...
            Xk_nsc(jointi.mtp.r*2,1),...
            Xk_nsc(jointi.trunk.ext*2,1),...
            Xk_nsc(jointi.trunk.ben*2,1),...
            Xk_nsc(jointi.trunk.rot*2,1),...
            Xk_nsc(jointi.hip_flex.l*2,1),...
            Xk_nsc(jointi.hip_add.l*2,1),...
            Xk_nsc(jointi.hip_rot.l*2,1),...
            Xk_nsc(jointi.knee.l*2,1),...
            Xk_nsc(jointi.ankle.l*2,1),...
            Xk_nsc(jointi.subt.l*2,1),...
            Xk_nsc(jointi.mtp.l*2,1)];      
        [lMTk_rtl,vMTk_rtl,MA_rtl] = f_lMT_vMT_dM(qin_rtl,qdotin_rtl);
              
        % Left leg
        MA.hip_flex.l   =  MA_rtl(mai(11).mus.l',11);
        MA.hip_add.l    =  MA_rtl(mai(12).mus.l',12);
        MA.hip_rot.l    =  MA_rtl(mai(13).mus.l',13);
        MA.knee.l       =  MA_rtl(mai(14).mus.l',14);
        MA.ankle.l      =  MA_rtl(mai(15).mus.l',15);  
        MA.subt.l       =  MA_rtl(mai(16).mus.l',16); 
        MA.trunk_ext    =  MA_rtl([mai(8).mus.l,mai(8).mus.r]',8);
        MA.trunk_ben    =  MA_rtl([mai(9).mus.l,mai(9).mus.r]',9);
        MA.trunk_rot    =  MA_rtl([mai(10).mus.l,mai(10).mus.r]',10);
        
        % Right leg
        MA.hip_flex.r   =  MA_rtl(mai(1).mus.r',1);
        MA.hip_add.r    =  MA_rtl(mai(2).mus.r',2);
        MA.hip_rot.r    =  MA_rtl(mai(3).mus.r',3);
        MA.knee.r       =  MA_rtl(mai(4).mus.r',4);
        MA.ankle.r      =  MA_rtl(mai(5).mus.r',5);
        MA.subt.r       =  MA_rtl(mai(6).mus.r',6);
        % Both legs
        lMTk_lr = [lMTk_rtl(:,1)];
        vMTk_lr = [vMTk_rtl(:,1)];   
        % Get muscle-tendon forces and derive Hill-equilibrium
        [Hilldiffk,FTk,Fcek,Fpassk,Fisok,vMmaxk,massMk] = ...
            f_forceEquilibrium_FtildeState_all_tendon_2(ak,...
                FTtildek.*scaling.FTtilde',dFTtildek.*scaling.dFTtilde,...
                lMTk_lr,vMTk_lr,tensions,aTendon,shift); 
            
        % Get metabolic energy rate if in the cost function   
        if W.E ~= 0    
            % Get muscle fiber lengths
            [~,lMtildek] = f_FiberLength_TendonForce_tendon(...
                FTtildek.*scaling.FTtilde',lMTk_lr,aTendon,shift); 
            % Get muscle fiber velocities
            [vMk,~] = f_FiberVelocity_TendonForce_tendon(...
                FTtildek.*scaling.FTtilde',dFTtildek.*scaling.dFTtilde,...
                lMTk_lr,vMTk_lr,aTendon,shift);
            % Get metabolic energy rate
            if mE == 0 % Bhargava et al. (2004)
                [e_tot,~,~,~,~,~] = fgetMetabolicEnergySmooth2004all_2(ak,ak,...
                    lMtildek,vMk,Fcek,Fpassk,massMk,pctsts,Fisok,...
                    MTparameters_m(1,:)',body_mass,10);
            elseif mE == 1 % Umberger et al. (2003)
                % vMtilde defined for this model as vM/lMopt
                vMtildeUmbk = vMk./(MTparameters_m(2,:)');
                [e_tot,~,~,~,~] = fgetMetabolicEnergySmooth2003all_2(...
                    ak,ak,lMtildek,vMtildeUmbk,vMk,Fcek,massMk,...
                    pctsts,vMmaxk,Fisok,body_mass,10);
            elseif mE == 2 % Umberger (2010)
                % vMtilde defined for this model as vM/lMopt
                vMtildeUmbk = vMk./(MTparameters_m(2,:)');
                [e_tot,~,~,~,~] = fgetMetabolicEnergySmooth2010all_2(...
                    ak,ak,lMtildek,vMtildeUmbk,vMk,Fcek,massMk,...
                    pctsts,vMmaxk,Fisok,body_mass,10);  
            elseif mE == 3 % Uchida et al. (2016)
                % vMtilde defined for this model as vM/lMopt
                vMtildeUmbk = vMk./(MTparameters_m(2,:)');
                [e_tot,~,~,~,~] = fgetMetabolicEnergySmooth2016all_2(...
                    ak,ak,lMtildek,vMtildeUmbk,vMk,Fcek,massMk,...
                    pctsts,vMmaxk,Fisok,body_mass,10);  
            elseif mE == 4 % Umberger (2010) treating muscle lengthening 
                % heat rate as Umberger et al. (2003)
                % vMtilde defined for this model as vM/lMopt
                vMtildeUmbk = vMk./(MTparameters_m(2,:)');
                [e_tot,~,~,~,~] = fgetMetabolicEnergySmooth2010all_hl_2(...
                    ak,ak,lMtildek,vMtildeUmbk,vMk,Fcek,massMk,...
                    pctsts,vMmaxk,Fisok,body_mass,10); 
            elseif mE == 5 % Umberger (2010) treating negative mechanical 
                % work as Umberger et al. (2003)
                % vMtilde defined for this model as vM/lMopt
            vMtildeUmbk = vMk./(MTparameters_m(2,:)');
            [e_tot,~,~,~,~] = fgetMetabolicEnergySmooth2010all_neg_2(...
                ak,ak,lMtildek,vMtildeUmbk,vMk,Fcek,massMk,...
                pctsts,vMmaxk,Fisok,body_mass,10); 
            end                
        end        
        % Get passive joint torques
        Tau_passk.hip.flex.l    = f_PassiveMoments(k_pass.hip.flex.l,...
            theta.pass.hip.flex.l,Xk_nsc(jointi.hip_flex.l*2-1,1),...
            Xk_nsc(jointi.hip_flex.l*2,1));
        Tau_passk.hip.flex.r    = f_PassiveMoments(k_pass.hip.flex.r,...
            theta.pass.hip.flex.r,Xk_nsc(jointi.hip_flex.r*2-1,1),...
            Xk_nsc(jointi.hip_flex.r*2,1));
        Tau_passk.hip.add.l     = f_PassiveMoments(k_pass.hip.add.l,...
            theta.pass.hip.add.l,Xk_nsc(jointi.hip_add.l*2-1,1),...
            Xk_nsc(jointi.hip_add.l*2,1));
        Tau_passk.hip.add.r     = f_PassiveMoments(k_pass.hip.add.r,...
            theta.pass.hip.add.r,Xk_nsc(jointi.hip_add.r*2-1,1),...
            Xk_nsc(jointi.hip_add.r*2,1));
        Tau_passk.hip.rot.l     = f_PassiveMoments(k_pass.hip.rot.l,...
            theta.pass.hip.rot.l,Xk_nsc(jointi.hip_rot.l*2-1,1),...
            Xk_nsc(jointi.hip_rot.l*2,1));
        Tau_passk.hip.rot.r     = f_PassiveMoments(k_pass.hip.rot.r,...
            theta.pass.hip.rot.r,Xk_nsc(jointi.hip_rot.r*2-1,1),...
            Xk_nsc(jointi.hip_rot.r*2,1));
        Tau_passk.knee.l        = f_PassiveMoments(k_pass.knee.l,...
            theta.pass.knee.l,Xk_nsc(jointi.knee.l*2-1,1),...
            Xk_nsc(jointi.knee.l*2,1));
        Tau_passk.knee.r        = f_PassiveMoments(k_pass.knee.r,...
            theta.pass.knee.r,Xk_nsc(jointi.knee.r*2-1,1),...
            Xk_nsc(jointi.knee.r*2,1));
        Tau_passk.ankle.l       = f_PassiveMoments(k_pass.ankle.l,...
            theta.pass.ankle.l,Xk_nsc(jointi.ankle.l*2-1,1),...
            Xk_nsc(jointi.ankle.l*2,1));
        Tau_passk.ankle.r       = f_PassiveMoments(k_pass.ankle.r,...
            theta.pass.ankle.r,Xk_nsc(jointi.ankle.r*2-1,1),...
            Xk_nsc(jointi.ankle.r*2,1));        
        Tau_passk.subt.l       = f_PassiveMoments(k_pass.subt.l,...
            theta.pass.subt.l,Xk_nsc(jointi.subt.l*2-1,1),...
            Xk_nsc(jointi.subt.l*2,1));
        Tau_passk.subt.r       = f_PassiveMoments(k_pass.subt.r,...
            theta.pass.subt.r,Xk_nsc(jointi.subt.r*2-1,1),...
            Xk_nsc(jointi.subt.r*2,1));
        Tau_passk.mtp.l       = f_PassiveMoments(k_pass.mtp.l,...
            theta.pass.mtp.l,Xk_nsc(jointi.mtp.l*2-1,1),...
            Xk_nsc(jointi.mtp.l*2,1));
        Tau_passk.mtp.r       = f_PassiveMoments(k_pass.mtp.r,...
            theta.pass.mtp.r,Xk_nsc(jointi.mtp.r*2-1,1),...
            Xk_nsc(jointi.mtp.r*2,1));
        Tau_passk.trunk.ext     = f_PassiveMoments(k_pass.trunk.ext,...
            theta.pass.trunk.ext,Xk_nsc(jointi.trunk.ext*2-1,1),...
            Xk_nsc(jointi.trunk.ext*2,1));
        Tau_passk.trunk.ben     = f_PassiveMoments(k_pass.trunk.ben,...
            theta.pass.trunk.ben,Xk_nsc(jointi.trunk.ben*2-1,1),...
            Xk_nsc(jointi.trunk.ben*2,1));
        Tau_passk.trunk.rot     = f_PassiveMoments(k_pass.trunk.rot,...
            theta.pass.trunk.rot,Xk_nsc(jointi.trunk.rot*2-1,1),...
            Xk_nsc(jointi.trunk.rot*2,1));        
        Tau_passk_all = [Tau_passk.hip.flex.l,Tau_passk.hip.flex.r,...
            Tau_passk.hip.add.l,Tau_passk.hip.add.r,...
            Tau_passk.hip.rot.l,Tau_passk.hip.rot.r,...
            Tau_passk.knee.l,Tau_passk.knee.r,Tau_passk.ankle.l,...
            Tau_passk.ankle.r,Tau_passk.subt.l,Tau_passk.subt.r,...
            Tau_passk.mtp.l,Tau_passk.mtp.r,...
            Tau_passk.trunk.ext,Tau_passk.trunk.ben,...
            Tau_passk.trunk.rot]';            
        
        % Torques mtp
        T_LinearPassive_mtp_l = f_LinearPassiveMTP(k_pass_mtp,...
            d_pass_mtp,Xk_nsc(jointi.mtp.l*2-1,1),...
            Xk_nsc(jointi.mtp.l*2,1));
        T_LinearPassive_mtp_r = f_LinearPassiveMTP(k_pass_mtp,...
            d_pass_mtp,Xk_nsc(jointi.mtp.r*2-1,1),...
            Xk_nsc(jointi.mtp.r*2,1));
        
        % Loop over collocation points
        Xk_nsc_end          = D(1)*Xk_nsc;
        FTtildek_nsc_end    = D(1)*FTtildek_nsc;
        ak_end              = D(1)*ak;
%         a_ak_end            = D(1)*a_ak;
        for j=1:d
            % Expression for the state derivatives at the collocation point
            xp_nsc          = C(1,j+1)*Xk_nsc;
            FTtildep_nsc    = C(1,j+1)*FTtildek_nsc;
            ap              = C(1,j+1)*ak;
%             a_ap            = C(1,j+1)*a_ak;
            for r=1:d
                xp_nsc       = xp_nsc + C(r+1,j+1)*Xkj_nsc{r};
                FTtildep_nsc = FTtildep_nsc + C(r+1,j+1)*FTtildekj_nsc{r};
                ap           = ap + C(r+1,j+1)*akj{r};
%                 a_ap         = a_ap + C(r+1,j+1)*a_akj{r};
            end 
            % Append collocation equations
            % Dynamic constraints are scaled using the same scale
            % factors as was used to scale the states
            % Activation dynamics (implicit formulation)  
            g       = {g{:}, (h*vAk.*scaling.vA - ap)./scaling.a};
            lbg     = [lbg; zeros(NMuscle,1)];
            ubg     = [ubg; zeros(NMuscle,1)]; 
            % Contraction dynamics (implicit formulation)            
            g       = {g{:}, (h*dFTtildek.*scaling.dFTtilde - ...
                FTtildep_nsc)./(scaling.FTtilde')};
            lbg     = [lbg; zeros(NMuscle,1)];
            ubg     = [ubg; zeros(NMuscle,1)];
            % Skeleton dynamics (implicit formulation)       
            xj_nsc  = [...
                Xkj_nsc{j}(2); Ak_nsc(1); Xkj_nsc{j}(4); Ak_nsc(2);...
                Xkj_nsc{j}(6); Ak_nsc(3); Xkj_nsc{j}(8); Ak_nsc(4);...
                Xkj_nsc{j}(10); Ak_nsc(5); Xkj_nsc{j}(12); Ak_nsc(6);...
                Xkj_nsc{j}(14); Ak_nsc(7); Xkj_nsc{j}(16); Ak_nsc(8);...
                Xkj_nsc{j}(18); Ak_nsc(9); Xkj_nsc{j}(20); Ak_nsc(10);...
                Xkj_nsc{j}(22); Ak_nsc(11); Xkj_nsc{j}(24); Ak_nsc(12);...
                Xkj_nsc{j}(26); Ak_nsc(13); Xkj_nsc{j}(28); Ak_nsc(14);...
                Xkj_nsc{j}(30); Ak_nsc(15); Xkj_nsc{j}(32); Ak_nsc(16);...
                Xkj_nsc{j}(34); Ak_nsc(17); Xkj_nsc{j}(36); Ak_nsc(18);...
                Xkj_nsc{j}(38); Ak_nsc(19); Xkj_nsc{j}(40); Ak_nsc(20);...
                Xkj_nsc{j}(42); Ak_nsc(21);Xkj_nsc{j}(44); Ak_nsc(22);...
                Xkj_nsc{j}(46); Ak_nsc(23);];
            g       = {g{:}, (h*xj_nsc - xp_nsc)./(scaling.QsQdots')};
            lbg     = [lbg; zeros(2*nq.all,1)];
            ubg     = [ubg; zeros(2*nq.all,1)];   
            % Add contribution to the end state
            Xk_nsc_end       = Xk_nsc_end + D(j+1)*Xkj_nsc{j};
            FTtildek_nsc_end = FTtildek_nsc_end + D(j+1)*FTtildekj_nsc{j};
            ak_end           = ak_end + D(j+1)*akj{j};  

            if W.E == 0
            J = J + 1/(dist_trav_tot)*(...
                W.A*B(j+1)      *(f_J92(akj{j}))*h + ...
                W.Ak*B(j+1)     *(f_J23(Ak(residuals_noarmsi,1)))*h + ... 
                W.passMom*B(j+1)*(f_J17(Tau_passk_all))*h + ...
                W.u*B(j+1)      *(f_J92(vAk))*h + ...
                W.u*B(j+1)      *(f_J92(dFTtildek))*h);
            elseif W.A == 0
            J = J + 1/(dist_trav_tot)*(...
                W.E*B(j+1)      *(f_J92exp(e_tot,exp_E))/body_mass*h + ...                
                W.Ak*B(j+1)     *(f_J23(Ak(residuals_noarmsi,1)))*h + ... 
                W.passMom*B(j+1)*(f_J17(Tau_passk_all))*h + ...
                W.u*B(j+1)      *(f_J92(vAk))*h + ...
                W.u*B(j+1)      *(f_J92(dFTtildek))*h); 
            elseif W.passMom == 0
                J = J + 1/(dist_trav_tot)*(...
                W.E*B(j+1)      *(f_J92exp(e_tot,exp_E))/body_mass*h + ...                
                W.A*B(j+1)      *(f_J92(akj{j}))*h + ...
                W.Ak*B(j+1)     *(f_J23(Ak(residuals_noarmsi,1)))*h + ... 
                W.u*B(j+1)      *(f_J92(vAk))*h + ...
                W.u*B(j+1)      *(f_J92(dFTtildek))*h);
            else                
            J = J + 1/(dist_trav_tot)*(...
                W.E*B(j+1)      *(f_J92exp(e_tot,exp_E))/body_mass*h + ...                
                W.A*B(j+1)      *(f_J92(akj{j}))*h + ...
                W.Ak*B(j+1)     *(f_J23(Ak(residuals_noarmsi,1)))*h + ... 
                W.passMom*B(j+1)*(f_J17(Tau_passk_all))*h + ...
                W.u*B(j+1)      *(f_J92(vAk))*h + ...
                W.u*B(j+1)      *(f_J92(dFTtildek))*h); 
            end
        end            
        % Call external function
        [Tk] = F([Xk_nsc;Ak_nsc]);           
        % Add path constraints
        % Null pelvis residuals
        g               = {g{:},Tk(ground_pelvisi,1)};
        lbg             = [lbg; zeros(nq.abs,1)];
        ubg             = [ubg; zeros(nq.abs,1)];  
        % Muscle-driven joint torques for the lower limbs and the trunk
        % Hip flexion, left
        Ft_hip_flex_l   = FTk(mai(11).mus.l',1);
        T_hip_flex_l    = f_T27(MA.hip_flex.l,Ft_hip_flex_l);
        g               = {g{:},Tk(jointi.hip_flex.l,1)-(T_hip_flex_l + ...
            Tau_passk.hip.flex.l - 0.1*Xk_nsc(jointi.hip_flex.l*2,1))};        
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];    
        % Hip flexion, right
        Ft_hip_flex_r   = FTk(mai(1).mus.r',1);
        T_hip_flex_r    = f_T27(MA.hip_flex.r,Ft_hip_flex_r);
        g               = {g{:},Tk(jointi.hip_flex.r,1)-(T_hip_flex_r + ...
            Tau_passk.hip.flex.r - 0.1*Xk_nsc(jointi.hip_flex.r*2,1))};        
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];   
        % Hip adduction, left
        Ft_hip_add_l    = FTk(mai(12).mus.l',1);
        T_hip_add_l     = f_T27(MA.hip_add.l,Ft_hip_add_l);
        g               = {g{:},Tk(jointi.hip_add.l,1)-(T_hip_add_l + ...
            Tau_passk.hip.add.l - 0.1*Xk_nsc(jointi.hip_add.l*2,1))};
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];    
        % Hip adduction, right
        Ft_hip_add_r    = FTk(mai(2).mus.r',1);
        T_hip_add_r     = f_T27(MA.hip_add.r,Ft_hip_add_r);
        g               = {g{:},Tk(jointi.hip_add.r,1)-(T_hip_add_r + ...
            Tau_passk.hip.add.r - 0.1*Xk_nsc(jointi.hip_add.r*2,1))};
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];  
        % Hip rotation, left
        Ft_hip_rot_l    = FTk(mai(13).mus.l',1);
        T_hip_rot_l     = f_T27(MA.hip_rot.l,Ft_hip_rot_l);
        g               = {g{:},Tk(jointi.hip_rot.l,1)-(T_hip_rot_l + ...
            Tau_passk.hip.rot.l - 0.1*Xk_nsc(jointi.hip_rot.l*2,1))};
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];    
        % Hip rotation, right
        Ft_hip_rot_r    = FTk(mai(3).mus.r',1);
        T_hip_rot_r     = f_T27(MA.hip_rot.r,Ft_hip_rot_r);
        g               = {g{:},Tk(jointi.hip_rot.r,1)-(T_hip_rot_r + ...
            Tau_passk.hip.rot.r - 0.1*Xk_nsc(jointi.hip_rot.r*2,1))};
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];   
        % Knee, left
        Ft_knee_l       = FTk(mai(14).mus.l',1);
        T_knee_l        = f_T13(MA.knee.l,Ft_knee_l);
        g               = {g{:},Tk(jointi.knee.l,1)-(T_knee_l + ...
            Tau_passk.knee.l - 0.1*Xk_nsc(jointi.knee.l*2,1))};
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];    
        % Knee, right
        Ft_knee_r       = FTk(mai(4).mus.r',1);
        T_knee_r        = f_T13(MA.knee.r,Ft_knee_r);
        g               = {g{:},Tk(jointi.knee.r,1)-(T_knee_r + ...
            Tau_passk.knee.r - 0.1*Xk_nsc(jointi.knee.r*2,1))};
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];    
        % Ankle, left
        Ft_ankle_l      = FTk(mai(15).mus.l',1);
        T_ankle_l       = f_T12(MA.ankle.l,Ft_ankle_l);
        g               = {g{:},Tk(jointi.ankle.l,1)-(T_ankle_l + ...
            Tau_passk.ankle.l - 0.1*Xk_nsc(jointi.ankle.l*2,1))};
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];    
        % Ankle, right
        Ft_ankle_r      = FTk(mai(5).mus.r',1);
        T_ankle_r       = f_T12(MA.ankle.r,Ft_ankle_r);
        g               = {g{:},Tk(jointi.ankle.r,1)-(T_ankle_r + ...
            Tau_passk.ankle.r - 0.1*Xk_nsc(jointi.ankle.r*2,1))};
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];
        % Subtalar, left
        Ft_subt_l       = FTk(mai(16).mus.l',1);
        T_subt_l        = f_T12(MA.subt.l,Ft_subt_l);
        g               = {g{:},(Tk(jointi.subt.l,1)-(T_subt_l + ...
            Tau_passk.subt.l - 0.1*Xk_nsc(jointi.subt.l*2,1)))};
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];    
        % Subtalar, right
        Ft_subt_r       = FTk(mai(6).mus.r',1);
        T_subt_r        = f_T12(MA.subt.r,Ft_subt_r);
        g               = {g{:},(Tk(jointi.subt.r,1)-(T_subt_r + ...
            Tau_passk.subt.r - 0.1*Xk_nsc(jointi.subt.r*2,1)))};
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];
        % Lumbar extension
        Ft_trunk_ext    = FTk([mai(8).mus.l,mai(8).mus.r]',1);
        T_trunk_ext     = f_T6(MA.trunk_ext,Ft_trunk_ext);
        g               = {g{:},Tk(jointi.trunk.ext,1)-(T_trunk_ext + ...
            Tau_passk.trunk.ext - 0.1*Xk_nsc(jointi.trunk.ext*2,1))};
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];
        % Lumbar bending
        Ft_trunk_ben    = FTk([mai(9).mus.l,mai(9).mus.r]',1);
        T_trunk_ben     = f_T6(MA.trunk_ben,Ft_trunk_ben);
        g               = {g{:},Tk(jointi.trunk.ben,1)-(T_trunk_ben + ...
            Tau_passk.trunk.ben - 0.1*Xk_nsc(jointi.trunk.ben*2,1))};
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];
        % Lumbar rotation
        Ft_trunk_rot    = FTk([mai(10).mus.l,mai(10).mus.r]',1);
        T_trunk_rot     = f_T6(MA.trunk_rot,Ft_trunk_rot);
        g               = {g{:},Tk(jointi.trunk.rot,1)-(T_trunk_rot + ...
            Tau_passk.trunk.rot - 0.1*Xk_nsc(jointi.trunk.rot*2,1))};
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];

        % Torques mtp
        g               = {g{:},Tk(jointi.mtp.l,1)/scaling.MtpTau - ...
            (T_LinearPassive_mtp_l + Tau_passk.mtp.l)/scaling.MtpTau};
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];
        g               = {g{:},Tk(jointi.mtp.r,1)/scaling.MtpTau - ...
            (T_LinearPassive_mtp_r + Tau_passk.mtp.r)/scaling.MtpTau};
        lbg             = [lbg; 0];
        ubg             = [ubg; 0];

        % Activation dynamics (implicit formulation)
        tact = 0.015;
        tdeact = 0.06;
        act1 = vAk*scaling.vA + ak./(ones(size(ak,1),1)*tdeact);
        act2 = vAk*scaling.vA + ak./(ones(size(ak,1),1)*tact);
        % act1
        g               = {g{:},act1};
        lbg             = [lbg; zeros(NMuscle,1)];
        ubg             = [ubg; inf*ones(NMuscle,1)]; 
        % act2
        g               = {g{:},act2};
        lbg             = [lbg; -inf*ones(NMuscle,1)];
        ubg             = [ubg; ones(NMuscle,1)./(ones(NMuscle,1)*tact)];        
        % Contraction dynamics (implicit formulation)
        g               = {g{:},Hilldiffk};
        lbg             = [lbg; zeros(NMuscle,1)];
        ubg             = [ubg; zeros(NMuscle,1)];  
        % Constraints to prevent parts of the skeleton to penetrate each
        % other.
        % Origins calcaneus (transv plane) at minimum 9 cm from each other.
        g               = {g{:},f_Jnn2(Tk(calcOr.r,1) - Tk(calcOr.l,1))};
        lbg             = [lbg; 0.0081];
        ubg             = [ubg; 4];   
        % Origins tibia (transv plane) at minimum 11 cm from each other.   
        g               = {g{:},f_Jnn2(Tk(tibiaOr.r,1) - Tk(tibiaOr.l,1))};
        lbg             = [lbg; 0.0121];
        ubg             = [ubg; 4]; 
        % We impose periodicity and symmetry and create some helper
        % variables to easily impose these constraints. Here we use these
        % variables when settings initial guesses of states at the end of
        % the current interval.
        % Muscles
        orderMusInv = [1:NMuscle];  
        % Lower limbs and trunk
        orderQsInv = [jointi.pelvis.tilt:2*jointi.pelvis.tz,...
            2*jointi.hip_flex.r-1:2*jointi.hip_rot.r,...
            2*jointi.hip_flex.l-1:2*jointi.hip_rot.l,...
            2*jointi.knee.r-1:2*jointi.knee.r,...
            2*jointi.knee.l-1:2*jointi.knee.l,...
            2*jointi.ankle.r-1:2*jointi.ankle.r,...
            2*jointi.ankle.l-1:2*jointi.ankle.l,...
            2*jointi.subt.r-1:2*jointi.subt.r,...
            2*jointi.subt.l-1:2*jointi.subt.l,...
            2*jointi.mtp.r-1:2*jointi.mtp.r,...
            2*jointi.mtp.l-1:2*jointi.mtp.l,...
            2*jointi.trunk.ext-1:2*jointi.trunk.rot];        
        orderQsOpp = [2*jointi.pelvis.list-1:2*jointi.pelvis.list,...   
            2*jointi.pelvis.rot-1:2*jointi.pelvis.rot,...
            2*jointi.pelvis.tz-1:2*jointi.pelvis.tz,...
            2*jointi.trunk.ben-1:2*jointi.trunk.ben,...
            2*jointi.trunk.rot-1:2*jointi.trunk.rot];
        orderQsCor = [jointi.pelvis.tilt:2*jointi.trunk.rot];
        
        % New NLP variables for states at end of interval
        if k ~= N-1
            % Muscle activations
            ak              = MX.sym(['a_' num2str(k+1)], NMuscle);
            w               = {w{:}, ak};
            lbw             = [lbw; bounds.a.lower'];
            ubw             = [ubw; bounds.a.upper'];
            w0              = [w0;  guess.a(k+2,:)'];
            % Muscle-tendon forces
            FTtildek        = MX.sym(['FTtilde_' num2str(k+1)], NMuscle);
            w               = {w{:}, FTtildek};
            lbw             = [lbw; bounds.FTtilde.lower'];
            ubw             = [ubw; bounds.FTtilde.upper'];
            w0              = [w0;  guess.FTtilde(k+2,:)'];    
            % Qs and Qdots
            w               = {w{:}, Xk{k+2,1}};
            lbw             = [lbw; bounds.QsQdots.lower'];
            ubw             = [ubw; bounds.QsQdots.upper']; 
            w0              = [w0;  guess.QsQdots(k+2,:)'];
        else % Periodicty            
            % Muscle activations
            ak              = MX.sym(['a_' num2str(k+1)], NMuscle);
            w               = {w{:}, ak};
            lbw             = [lbw; bounds.a.lower'];
            ubw             = [ubw; bounds.a.upper'];            
            w0              = [w0;  guess.a(1,orderMusInv)'];
            % Muscle-tendon forces
            FTtildek        = MX.sym(['FTtilde_' num2str(k+1)], NMuscle);
            w               = {w{:}, FTtildek};
            lbw             = [lbw; bounds.FTtilde.lower'];
            ubw             = [ubw; bounds.FTtilde.upper'];
            w0              = [w0;  guess.FTtilde(1,orderMusInv)'];    
            % Qs and Qdots
            w               = {w{:}, Xk{k+2,1}};
            lbw             = [lbw; bounds.QsQdots.lower'];
            ubw             = [ubw; bounds.QsQdots.upper'];
            % For "bilateral" joints, we invert right and left
            inv_X           = guess.QsQdots(1,orderQsCor);           
            dx = guess.QsQdots(end,2*jointi.pelvis.tx-1) - ...
                guess.QsQdots(end-1,2*jointi.pelvis.tx-1);
            inv_X(2*jointi.pelvis.tx-1) = ...
                guess.QsQdots(end,2*jointi.pelvis.tx-1) + dx;            
            w0                = [w0;  inv_X'];   
        end
        
        % Rescale variables to impose equality constraints
        Xk_end = (Xk_nsc_end)./scaling.QsQdots';
        FTtildek_end = (FTtildek_nsc_end)./scaling.FTtilde';
        % Add equality constraints (next interval starts with end values of 
        % states from previous interval)
        g   = {g{:}, Xk_end-Xk{k+2,1}, FTtildek_end-FTtildek, ...
            ak_end-ak};
        lbg = [lbg; zeros(2*nq.all + NMuscle + NMuscle,1)];
        ubg = [ubg; zeros(2*nq.all + NMuscle + NMuscle,1)];         
    end
    % Additional path constraints
    % Periodicity of the states
    % Qs and Qdots
    QsInvA = [jointi.pelvis.tilt:2*jointi.pelvis.tilt,...
        2*jointi.pelvis.tx,2*jointi.pelvis.ty-1:2*jointi.pelvis.ty,...
        2*jointi.hip_flex.l-1:2*jointi.trunk.ext]';
    QsInvB = [jointi.pelvis.tilt:2*jointi.pelvis.tilt,...
        2*jointi.pelvis.tx,2*jointi.pelvis.ty-1:2*jointi.pelvis.ty,...
        2*jointi.hip_flex.r-1:2*jointi.hip_rot.r,...
        2*jointi.hip_flex.l-1:2*jointi.hip_rot.l,...
        2*jointi.knee.r-1:2*jointi.knee.r,...
        2*jointi.knee.l-1:2*jointi.knee.l,...
        2*jointi.ankle.r-1:2*jointi.ankle.r,...
        2*jointi.ankle.l-1:2*jointi.ankle.l,...
        2*jointi.subt.r-1:2*jointi.subt.r,...
        2*jointi.subt.l-1:2*jointi.subt.l,...
        2*jointi.mtp.r-1:2*jointi.mtp.r,...
        2*jointi.mtp.l-1:2*jointi.mtp.l,...
        2*jointi.trunk.ext-1,2*jointi.trunk.ext]';
    orderQsCor2 = [jointi.pelvis.tilt:2*jointi.pelvis.rot,...
        2*jointi.pelvis.tx:2*jointi.trunk.rot];
    g   = {g{:}, Xk_end(orderQsCor2)-X0(orderQsCor2,1)};
    lbg = [lbg; zeros(length(orderQsCor2),1)];
    ubg = [ubg; zeros(length(orderQsCor2),1)];     
     
    % Muscle activations
    g   = {g{:}, ak_end-a0(orderMusInv,1)};
    lbg = [lbg; zeros(NMuscle,1)];
    ubg = [ubg; zeros(NMuscle,1)];
    % Muscle-tendon forces
    g   = {g{:}, FTtildek_end-FTtilde0(orderMusInv,1)};
    lbg = [lbg; zeros(NMuscle,1)];
    ubg = [ubg; zeros(NMuscle,1)];

    % Average speed
    vel_aver_tot = dist_trav_tot/tf; 
    g   = {g{:}, vel_aver_tot - v_tgt};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    
    % Assert bounds / IG
    % Lower bounds smaller than upper bounds    
    assert_bw = isempty(find(lbw <= ubw == 0,1)); % Design variables
    assert_bg = isempty(find(lbg <= ubg == 0,1)); % Constraints
    % Design variables between -1 and 1
    assert_bwl = isempty(find(lbw < -1 == 1,1));
    assert_bwu = isempty(find(1 < ubw == 1,1));   
    % Initial guess within bounds
    assert_w0_ubw = isempty(find(w0 <= ubw == 0,1));
    assert_w0_lbw = isempty(find(lbw <= w0 == 0,1));
                
    % Create NLP solver
    prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
    options.ipopt.hessian_approximation = 'limited-memory';
    options.ipopt.mu_strategy      = 'adaptive';
    options.ipopt.max_iter = 3000;
    options.ipopt.tol = 1*10^(-tol_ipopt);
    solver = nlpsol('solver', 'ipopt', prob, options);
    % Create and save diary
    p = mfilename('fullpath');
    [~,namescript,~] = fileparts(p);
    pathresults = [pathRepo,'/Results/',subject];
    if ~(exist([pathresults,'/',namescript],'dir')==7)
        mkdir(pathresults,namescript);
    end
    if (exist([pathresults,'/',namescript,'/D',savename],'file')==2)
        delete ([pathresults,'/',namescript,'/D',savename])
    end 
    diary([pathresults,'/',namescript,'/D',savename]);  
    % Data-informed (full solution at closest speed) initial guess    
    if IGm == 4    
        load([pathresults,'/',namescript,'/w',savename_ig]);
        w0 = w_opt; 
        % From 1.43 m s-1, the bounds on some Qs and Qdots are changed to
        % allow for generation of running motions. Therefore, we need to
        % adjust for that change when using the data-informed (full solution at 
        % closest speed) initial guess. This is only for one trial, the one
        % at 1.43 m s-1 that uses as initial guess the solution from the
        % one at 1.33 m s-1.
        if v_tgt == 1.43
            % We first extract the Qs and Qdots from w_opt.       
            % All optimized design variables are saved in a single column vector      
            % Number of design variables
            NControls = NMuscle+NMuscle+nq.all;
            NStates = NMuscle+NMuscle+2*nq.all;            
            NParameters = 1;    
            % In the loop
            Nwl = NControls+d*(NStates)+NStates;
            % In total
            Nw = NParameters+NStates+N*Nwl;
            % Before the variable corresponding to the first collocation point
            Nwm = NParameters+NStates+NControls;
            % Mesh points
            % Qs and Qdots
            q_w0 = zeros(N+1,nq.all);
            qdot_w0 = zeros(N+1,nq.all);
            count = 0;
            for i = 1:2:2*nq.all
                count = count +1;
                q_w0(:,count) = w_opt(NParameters+NMuscle+NMuscle+i:Nwl:Nw);
                qdot_w0(:,count) =w_opt(NParameters+NMuscle+NMuscle+i+1:Nwl:Nw);
            end
            % Collocation points
            % Qs and Qdots
            q_w0_col=zeros(N*d,nq.all);
            q_dot_w0_col=zeros(N*d,nq.all);
            nqi_col = 1:2:2*nq.all;
            temp_h = Nwm+d*NMuscle+d*NMuscle;
            for nqi=1:nq.all
                nqi_q = nqi_col(nqi);
                q_w0_col(1:d:end,nqi) = w_opt(temp_h+nqi_q:Nwl:Nw);                   
                q_w0_col(2:d:end,nqi) = w_opt(temp_h+2*nq.all+nqi_q:Nwl:Nw);  
                q_w0_col(3:d:end,nqi) = ...
                    w_opt(temp_h+2*nq.all+2*nq.all+nqi_q:Nwl:Nw);  
                q_dot_w0_col(1:d:end,nqi) = w_opt(temp_h+nqi_q+1:Nwl:Nw);   
                q_dot_w0_col(2:d:end,nqi) = ...
                    w_opt(temp_h+2*nq.all+nqi_q+1:Nwl:Nw);  
                q_dot_w0_col(3:d:end,nqi) = ...
                    w_opt(temp_h+2*nq.all+2*nq.all+nqi_q+1:Nwl:Nw);
            end     
            % We then unscale using scaling factors used for speeds lower
            % than 1.43 m s-1
            [~,scaling_walk] = getBounds_run(Qs_walk,NMuscle,nq,jointi,1.33);
            % Qs
            q_w0_unsc = q_w0.*repmat(scaling_walk.Qs,size(q_w0,1),1); 
            q_w0_col_unsc =q_w0_col.*repmat(scaling_walk.Qs,size(q_w0_col,1),1);
            % Qdots
            qdot_w0_unsc =qdot_w0.*repmat(scaling_walk.Qdots,size(qdot_w0,1),1); 
            qdot_w0_col_unsc = ...
                q_dot_w0_col.*repmat(scaling_walk.Qdots,size(q_dot_w0_col,1),1);
            % We then rescale using scaling factors used for speeds larger
            % than 1.33 m s-1
            % Qs
            q_w0_sc = q_w0_unsc./repmat(scaling.Qs,size(q_w0,1),1); 
            q_w0_col_sc = q_w0_col_unsc./repmat(scaling.Qs,size(q_w0_col,1),1);
            % Qdots
            qdot_w0_sc = qdot_w0_unsc./repmat(scaling.Qdots,size(qdot_w0,1),1); 
            qdot_w0_col_sc = ...
                qdot_w0_col_unsc./repmat(scaling.Qdots,size(q_dot_w0_col,1),1);
            % We then put the scaleds variables back in w0
            % Mesh points
            count = 0;
            for i = 1:2:2*nq.all
                count = count +1;
                w0(NParameters+NMuscle+NMuscle+i:Nwl:Nw) = q_w0_sc(:,count);
                w0(NParameters+NMuscle+NMuscle+i+1:Nwl:Nw) =qdot_w0_sc(:,count);
            end
            % Collocation points
            for nqi=1:nq.all
                nqi_q = nqi_col(nqi);
                 w0(temp_h+nqi_q:Nwl:Nw) = q_w0_col_sc(1:d:end,nqi);   
                 w0(temp_h+2*nq.all+nqi_q:Nwl:Nw) = q_w0_col_sc(2:d:end,nqi);  
                 w0(temp_h+2*nq.all+2*nq.all+nqi_q:Nwl:Nw) =...
                     q_w0_col_sc(3:d:end,nqi);  
                 w0(temp_h+nqi_q+1:Nwl:Nw) = qdot_w0_col_sc(1:d:end,nqi);   
                 w0(temp_h+2*nq.all+nqi_q+1:Nwl:Nw) = ...
                     qdot_w0_col_sc(2:d:end,nqi);  
                 w0(temp_h+2*nq.all+2*nq.all+nqi_q+1:Nwl:Nw) = ...
                     qdot_w0_col_sc(3:d:end,nqi);
            end   
        end
        clear w_opt;
    end
    % Solve problem
    sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
        'lbg', lbg, 'ubg', ubg);    
    diary off
    % Extract results
    w_opt = full(sol.x);
    g_opt = full(sol.g);  
    % Create setup
    setup.tolerance.ipopt = tol_ipopt;
    setup.bounds = bounds;
    setup.scaling = scaling;
    setup.guess = guess;
    setup.lbw = lbw;
    setup.ubw = ubw;
    % Save results and setup
    save([pathresults,'/',namescript,'/w',savename],'w_opt');
    save([pathresults,'/',namescript,'/g',savename],'g_opt');
    save([pathresults,'/',namescript,'/s',savename],'setup');
end

%% Analyze results
if analyseResults
    %% Load results
    if loadResults
        p = mfilename('fullpath');
        [~,namescript,~] = fileparts(p);
        pathresults = [pathRepo,'/Results/',subject];
        load([pathresults,'/',namescript,'/w',savename]);
        load([pathresults,'/',namescript,'/g',savename]);
        load([pathresults,'/',namescript,'/s',savename]);
    end  
    
    %% Extract results
    % All optimized design variables are saved in a single column vector      
    % Number of design variables
    NControls = NMuscle+NMuscle+nq.all;
    NStates = NMuscle+NMuscle+2*nq.all;    
    NParameters = 1;    
    % In the loop
    Nwl = NControls+d*(NStates)+NStates;
    % In total
    Nw = NParameters+NStates+N*Nwl;
    % Before the variable corresponding to the first collocation point
    Nwm = NParameters+NStates+NControls;
    % Here we extract the results and re-organize them for analysis  
    % Static parameters
    tf_opt  = w_opt(1:NParameters);
    % Mesh points
    % Muscle activations and muscle-tendon forces
    a_opt = zeros(N+1,NMuscle);
    FTtilde_opt = zeros(N+1,NMuscle);
    for i = 1:NMuscle
        a_opt(:,i) = w_opt(NParameters+i:Nwl:Nw);
        FTtilde_opt(:,i) = w_opt(NParameters+NMuscle+i:Nwl:Nw);
    end
    % Qs and Qdots
    q_opt = zeros(N+1,nq.all);
    qdot_opt = zeros(N+1,nq.all);
    count = 0;
    for i = 1:2:2*nq.all
        count = count +1;
        q_opt(:,count) = w_opt(NParameters+NMuscle+NMuscle+i:Nwl:Nw);
        qdot_opt(:,count) = w_opt(NParameters+NMuscle+NMuscle+i+1:Nwl:Nw);
    end

    % Time derivative of muscle activations and muscle-tendon forces
    vA_opt = zeros(N,NMuscle);
    dFTtilde_opt = zeros(N,NMuscle);
    for i = 1:NMuscle
        vA_opt(:,i) = w_opt(NParameters+NStates+i:Nwl:Nw);
        dFTtilde_opt(:,i) = w_opt(NParameters+NStates+NMuscle+i:Nwl:Nw);
    end
    % Time derivative of joint velocities
    qdotdot_opt = zeros(N,nq.all);
    for i = 1:nq.all
        qdotdot_opt(:,i) = ...
            w_opt(NParameters+NStates+NMuscle+NMuscle+i:Nwl:Nw);
    end

    % Collocation points
    % Muscle activations
    a_opt_ext=zeros(N*(d+1)+1,NMuscle);
    a_opt_ext(1:(d+1):end,:)= a_opt;
    for nmusi=1:NMuscle
        a_opt_ext(2:(d+1):end,nmusi) = w_opt(Nwm+nmusi:Nwl:Nw);
        a_opt_ext(3:(d+1):end,nmusi) = w_opt(Nwm+NMuscle+nmusi:Nwl:Nw);
        a_opt_ext(4:(d+1):end,nmusi) = ...
            w_opt(Nwm+NMuscle+NMuscle+nmusi:Nwl:Nw);
    end
    % Muscle activations at collocation points only
    a_opt_ext_col = zeros(N*d,NMuscle); 
    for nmusi=1:NMuscle
        a_opt_ext_col(1:d:end,nmusi) = w_opt(Nwm+nmusi:Nwl:Nw);
        a_opt_ext_col(2:d:end,nmusi) = w_opt(Nwm+NMuscle+nmusi:Nwl:Nw);
        a_opt_ext_col(3:d:end,nmusi) = ...
            w_opt(Nwm+NMuscle+NMuscle+nmusi:Nwl:Nw);   
    end    
    % Muscle-tendon forces
    FTtilde_opt_ext=zeros(N*(d+1)+1,NMuscle);
    FTtilde_opt_ext(1:(d+1):end,:)= FTtilde_opt;
    for nmusi=1:NMuscle
        FTtilde_opt_ext(2:(d+1):end,nmusi) = ...
            w_opt(Nwm+d*NMuscle+nmusi:Nwl:Nw);
        FTtilde_opt_ext(3:(d+1):end,nmusi) = ...
            w_opt(Nwm+d*NMuscle+NMuscle+nmusi:Nwl:Nw);
        FTtilde_opt_ext(4:(d+1):end,nmusi) = ...
            w_opt(Nwm+d*NMuscle+NMuscle+NMuscle+nmusi:Nwl:Nw);
    end
    % Qs and Qdots
    q_opt_ext=zeros(N*(d+1)+1,nq.all);
    q_opt_ext(1:(d+1):end,:)= q_opt;
    q_dot_opt_ext=zeros(N*(d+1)+1,nq.all);
    q_dot_opt_ext(1:(d+1):end,:)= qdot_opt;
    nqi_col = 1:2:2*nq.all;
    for nqi=1:nq.all
        nqi_q = nqi_col(nqi);
        q_opt_ext(2:(d+1):end,nqi) = w_opt(Nwm+d*NMuscle+...
            d*NMuscle+nqi_q:Nwl:Nw);   
        q_opt_ext(3:(d+1):end,nqi) = w_opt(Nwm+d*NMuscle+...
            d*NMuscle+2*nq.all+nqi_q:Nwl:Nw);  
        q_opt_ext(4:(d+1):end,nqi) = w_opt(Nwm+d*NMuscle+...
            d*NMuscle+2*nq.all+2*nq.all+nqi_q:Nwl:Nw);  
        q_dot_opt_ext(2:(d+1):end,nqi) = w_opt(Nwm+d*NMuscle+...
            d*NMuscle+nqi_q+1:Nwl:Nw);   
        q_dot_opt_ext(3:(d+1):end,nqi) = w_opt(Nwm+d*NMuscle+...
            d*NMuscle+2*nq.all+nqi_q+1:Nwl:Nw);  
        q_dot_opt_ext(4:(d+1):end,nqi) = w_opt(Nwm+d*NMuscle+...
            d*NMuscle+2*nq.all+2*nq.all+nqi_q+1:Nwl:Nw);
    end
    
    %% Unscale results
    % States at mesh points
    % Qs (1:N-1)
    q_opt_unsc.rad = q_opt(1:end-1,:).*repmat(...
        scaling.Qs,size(q_opt(1:end-1,:),1),1); 
    % Convert in degrees
    q_opt_unsc.deg = q_opt_unsc.rad;
    q_opt_unsc.deg(:,[1:3,7:end]) = q_opt_unsc.deg(:,[1:3,7:end]).*180/pi;
    % Qs (1:N)
    q_opt_unsc_all.rad = q_opt(1:end,:).*repmat(...
        scaling.Qs,size(q_opt(1:end,:),1),1); 
    % Convert in degrees
    q_opt_unsc_all.deg = q_opt_unsc_all.rad;
    q_opt_unsc_all.deg(:,[1:3,7:end]) = ...
        q_opt_unsc_all.deg(:,[1:3,7:end]).*180/pi;       
    % Qdots (1:N-1)
    qdot_opt_unsc.rad = qdot_opt(1:end-1,:).*repmat(...
        scaling.Qdots,size(qdot_opt(1:end-1,:),1),1);
    % Convert in degrees
    qdot_opt_unsc.deg = qdot_opt_unsc.rad;
    qdot_opt_unsc.deg(:,[1:3,7:end]) = ...
        qdot_opt_unsc.deg(:,[1:3,7:end]).*180/pi;
    % Qdots (1:N)
    qdot_opt_unsc_all.rad = qdot_opt(1:end,:).*repmat(...
        scaling.Qdots,size(qdot_opt(1:end,:),1),1); 
    % Muscle activations
    a_opt_unsc = a_opt(1:end-1,:).*repmat(...
        scaling.a,size(a_opt(1:end-1,:),1),size(a_opt,2));
    % Muscle-tendon forces
    FTtilde_opt_unsc = FTtilde_opt(1:end-1,:).*repmat(...
        scaling.FTtilde,size(FTtilde_opt(1:end-1,:),1),1);

    % Controls at mesh points
    % Time derivative of Qdots
    qdotdot_opt_unsc.rad = ...
        qdotdot_opt.*repmat(scaling.Qdotdots,size(qdotdot_opt,1),1);
    % Convert in degrees
    qdotdot_opt_unsc.deg = qdotdot_opt_unsc.rad;
    qdotdot_opt_unsc.deg(:,[1:3,7:end]) = ...
        qdotdot_opt_unsc.deg(:,[1:3,7:end]).*180/pi;
    % Time derivative of muscle activations (states)
    vA_opt_unsc = vA_opt.*repmat(scaling.vA,size(vA_opt,1),size(vA_opt,2));
    tact = 0.015;
    tdeact = 0.06;
    % Get muscle excitations from time derivative of muscle activations
    e_opt_unsc = computeExcitationRaasch(a_opt_unsc,vA_opt_unsc,...
        ones(1,NMuscle)*tdeact,ones(1,NMuscle)*tact);
    % Time derivative of muscle-tendon forces
    dFTtilde_opt_unsc = dFTtilde_opt.*repmat(...
        scaling.dFTtilde,size(dFTtilde_opt,1),size(dFTtilde_opt,2));
    
    %% Time grid    
    % Mesh points
    tgrid = linspace(0,tf_opt,N+1);
    dtime = zeros(1,d+1);
    for i=1:4
        dtime(i)=tau_root(i)*(tf_opt/N);
    end
    % Mesh points and collocation points
    tgrid_ext = zeros(1,(d+1)*N+1);
    for i=1:N
        tgrid_ext(((i-1)*4+1):1:i*4)=tgrid(i)+dtime;
    end
    tgrid_ext(end)=tf_opt;     
 
    %% Joint torques and ground reaction forces at optimal solution
    Xk_Qs_Qdots_opt             = zeros(N,2*nq.all);
    Xk_Qs_Qdots_opt(:,1:2:end)  = q_opt_unsc.rad;
    Xk_Qs_Qdots_opt(:,2:2:end)  = qdot_opt_unsc.rad;
    Xk_Qdotdots_opt             = qdotdot_opt_unsc.rad;
    out_res_opt = zeros(N,nq.all+NGRF+NcalcOrall);
    for i = 1:N
        [res] = F1([Xk_Qs_Qdots_opt(i,:)';Xk_Qdotdots_opt(i,:)']);
        out_res_opt(i,:) = full(res);    
    end
    GRF_opt_unsc = out_res_opt(:,GRFi.all);
    Tauk_out        = out_res_opt(:,residualsi);

    %% Stride length and width  
    % For the stride length we also need the values at the end of the
    % interval so N+1 where states but not controls are defined
    Xk_Qs_Qdots_opt_all = zeros(N+1,2*size(q_opt_unsc_all.rad,2));
    Xk_Qs_Qdots_opt_all(:,1:2:end)  = q_opt_unsc_all.rad;
    Xk_Qs_Qdots_opt_all(:,2:2:end)  = qdot_opt_unsc_all.rad;
    % We just want to extract the positions of the calcaneus origins so we
    % do not really care about Qdotdot that we set to 0
    Xk_Qdotdots_opt_all = zeros(N+1,size(q_opt_unsc_all.rad,2));
    out_res_opt_all = zeros(N+1,nq.all+NGRF+NcalcOrall);
    for i = 1:N+1
        [res] = F1([Xk_Qs_Qdots_opt_all(i,:)';Xk_Qdotdots_opt_all(i,:)']);
        out_res_opt_all(i,:) = full(res);    
    end
    % The stride length is the distance covered by the calcaneus origin
    % Right leg
    dist_r = sqrt(f_Jnn3(out_res_opt_all(end,calcOrall.r)-...
        out_res_opt_all(1,calcOrall.r)));
    % Left leg
    dist_l = sqrt(f_Jnn3(out_res_opt_all(end,calcOrall.l)-...
        out_res_opt_all(1,calcOrall.l)));
    % The total stride length is the sum of the right and left stride 
    % lengths after a half gait cycle, since we assume symmetry
    StrideLength_opt = full(dist_r + dist_l);    
    % The stride width is the medial distance between the calcaneus origins
    StepWidth_opt = full(abs(out_res_opt_all(:,calcOrall.r(3)) - ...
        out_res_opt_all(:,calcOrall.l(3))));
    stride_width_mean = mean(StepWidth_opt);
    stride_width_std = std(StepWidth_opt);
    calc_Orig_r = out_res_opt_all(:,calcOrall.r);
    calc_Orig_l = out_res_opt_all(:,calcOrall.l);
    
    %% Passive joint torques at optimal solution
    Tau_pass_opt_all = zeros(N,17);
    for i = 1:N    
        Tau_pass_opt.hip.flex.l    = f_PassiveMoments(k_pass.hip.flex.l,...
           theta.pass.hip.flex.l,Xk_Qs_Qdots_opt(i,jointi.hip_flex.l*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.hip_flex.l*2));
        Tau_pass_opt.hip.flex.r    = f_PassiveMoments(k_pass.hip.flex.r,...
           theta.pass.hip.flex.r,Xk_Qs_Qdots_opt(i,jointi.hip_flex.r*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.hip_flex.r*2));
        Tau_pass_opt.hip.add.l     = f_PassiveMoments(k_pass.hip.add.l,...
           theta.pass.hip.add.l,Xk_Qs_Qdots_opt(i,jointi.hip_add.l*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.hip_add.l*2));
        Tau_pass_opt.hip.add.r     = f_PassiveMoments(k_pass.hip.add.r,...
           theta.pass.hip.add.r,Xk_Qs_Qdots_opt(i,jointi.hip_add.r*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.hip_add.r*2));
        Tau_pass_opt.hip.rot.l     = f_PassiveMoments(k_pass.hip.rot.l,...
           theta.pass.hip.rot.l,Xk_Qs_Qdots_opt(i,jointi.hip_rot.l*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.hip_rot.l*2));
        Tau_pass_opt.hip.rot.r     = f_PassiveMoments(k_pass.hip.rot.r,...
           theta.pass.hip.rot.r,Xk_Qs_Qdots_opt(i,jointi.hip_rot.r*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.hip_rot.r*2));
        Tau_pass_opt.knee.l        = f_PassiveMoments(k_pass.knee.l,...
           theta.pass.knee.l,Xk_Qs_Qdots_opt(i,jointi.knee.l*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.knee.l*2));
        Tau_pass_opt.knee.r        = f_PassiveMoments(k_pass.knee.r,...
           theta.pass.knee.r,Xk_Qs_Qdots_opt(i,jointi.knee.r*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.knee.r*2));
        Tau_pass_opt.ankle.l       = f_PassiveMoments(k_pass.ankle.l,...
           theta.pass.ankle.l,Xk_Qs_Qdots_opt(i,jointi.ankle.l*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.ankle.l*2));
        Tau_pass_opt.ankle.r       = f_PassiveMoments(k_pass.ankle.r,...
           theta.pass.ankle.r,Xk_Qs_Qdots_opt(i,jointi.ankle.r*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.ankle.r*2));
        Tau_pass_opt.subt.l       = f_PassiveMoments(k_pass.subt.l,...
           theta.pass.subt.l,Xk_Qs_Qdots_opt(i,jointi.subt.l*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.subt.l*2));
        Tau_pass_opt.subt.r       = f_PassiveMoments(k_pass.subt.r,...
           theta.pass.subt.r,Xk_Qs_Qdots_opt(i,jointi.subt.r*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.subt.r*2));
        Tau_pass_opt.mtp.l       = f_PassiveMoments(k_pass.mtp.l,...
           theta.pass.mtp.l,Xk_Qs_Qdots_opt(i,jointi.mtp.l*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.mtp.l*2));
        Tau_pass_opt.mtp.r       = f_PassiveMoments(k_pass.mtp.r,...
           theta.pass.mtp.r,Xk_Qs_Qdots_opt(i,jointi.mtp.r*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.mtp.r*2));
        Tau_pass_opt.trunk.ext     = f_PassiveMoments(k_pass.trunk.ext,...
           theta.pass.trunk.ext,Xk_Qs_Qdots_opt(i,jointi.trunk.ext*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.trunk.ext*2));
        Tau_pass_opt.trunk.ben     = f_PassiveMoments(k_pass.trunk.ben,...
           theta.pass.trunk.ben,Xk_Qs_Qdots_opt(i,jointi.trunk.ben*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.trunk.ben*2));
        Tau_pass_opt.trunk.rot     = f_PassiveMoments(k_pass.trunk.rot,...
           theta.pass.trunk.rot,Xk_Qs_Qdots_opt(i,jointi.trunk.rot*2-1),...
           Xk_Qs_Qdots_opt(i,jointi.trunk.rot*2));        
        Tau_pass_opt_all(i,:) = full([Tau_pass_opt.hip.flex.l,...
           Tau_pass_opt.hip.add.l,Tau_pass_opt.hip.rot.l,...             
           Tau_pass_opt.hip.flex.r,Tau_pass_opt.hip.add.r,...
           Tau_pass_opt.hip.rot.r,Tau_pass_opt.knee.l,...
           Tau_pass_opt.knee.r,Tau_pass_opt.ankle.l,...
           Tau_pass_opt.ankle.r,Tau_pass_opt.subt.l,...
           Tau_pass_opt.subt.r,Tau_pass_opt.mtp.l,...
           Tau_pass_opt.mtp.r,Tau_pass_opt.trunk.ext,...
           Tau_pass_opt.trunk.ben,Tau_pass_opt.trunk.rot]);
    end    
    
    %% Assert average speed    
    dist_trav_opt = q_opt_ext(end,jointi.pelvis.tx)*...
        scaling.Qs(jointi.pelvis.tx) - q_opt_ext(1,jointi.pelvis.tx)*...
        scaling.Qs(jointi.pelvis.tx); % distance traveled
    time_elaps_opt = tf_opt; % time elapsed
    vel_aver_opt = dist_trav_opt/time_elaps_opt; 
    % assert_v_tg should be 0
    assert_v_tg = abs(vel_aver_opt-v_tgt);
    
    %% Muscle-tendon force and fiber lengths, velocities at optimal solution
    FT_opt_all_new = zeros(N,NMuscle);
    Fce_opt_all_new = zeros(N,NMuscle);
    Fpass_opt_all_new = zeros(N,NMuscle);
    Fiso_opt_all_new = zeros(N,NMuscle);
    lM_opt_all_new = zeros(N,NMuscle);
    vM_opt_all_new = zeros(N,NMuscle);
    lMT_opt_all_new = zeros(N,NMuscle);
    vMT_opt_all_new = zeros(N,NMuscle);
    for k=1:N
        % Both legs
        qin_rtl_opt_all = [Xk_Qs_Qdots_opt(k,jointi.hip_flex.r*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.hip_add.r*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.hip_rot.r*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.knee.r*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.ankle.r*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.subt.r*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.mtp.r*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.ext*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.ben*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.rot*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.hip_flex.l*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.hip_add.l*2-1), ...
            Xk_Qs_Qdots_opt(k,jointi.hip_rot.l*2-1), ...
            Xk_Qs_Qdots_opt(k,jointi.knee.l*2-1), ...
            Xk_Qs_Qdots_opt(k,jointi.ankle.l*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.subt.l*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.mtp.l*2-1)];  
        qdotin_rtl_opt_all = [Xk_Qs_Qdots_opt(k,jointi.hip_flex.r*2),...
            Xk_Qs_Qdots_opt(k,jointi.hip_add.r*2),...
            Xk_Qs_Qdots_opt(k,jointi.hip_rot.r*2),...
            Xk_Qs_Qdots_opt(k,jointi.knee.r*2),...
            Xk_Qs_Qdots_opt(k,jointi.ankle.r*2),...
            Xk_Qs_Qdots_opt(k,jointi.subt.r*2),...
            Xk_Qs_Qdots_opt(k,jointi.mtp.r*2),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.ext*2),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.ben*2),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.rot*2),...
            Xk_Qs_Qdots_opt(k,jointi.hip_flex.l*2),...
            Xk_Qs_Qdots_opt(k,jointi.hip_add.l*2),...
            Xk_Qs_Qdots_opt(k,jointi.hip_rot.l*2),...
            Xk_Qs_Qdots_opt(k,jointi.knee.l*2),...
            Xk_Qs_Qdots_opt(k,jointi.ankle.l*2),...
            Xk_Qs_Qdots_opt(k,jointi.subt.l*2),...
            Xk_Qs_Qdots_opt(k,jointi.mtp.l*2)];      
        [lMTk_rtl_opt_all,vMTk_rtl_opt_all,~] = ...
            f_lMT_vMT_dM(qin_rtl_opt_all,qdotin_rtl_opt_all);
                        
        % Both legs
        lMTk_lr_opt_all = ...
            [lMTk_rtl_opt_all(:,1)];
        vMTk_lr_opt_all = ...
            [vMTk_rtl_opt_all(:,1)];        
        % Muscle forces at optimal solution
        [~,FT_opt_all_a,Fce_opt_all_a,Fpass_opt_all_a,Fiso_opt_all_a,~,...
            ~] = f_forceEquilibrium_FtildeState_all_tendon_2(...
                a_opt_unsc(k,:)',FTtilde_opt_unsc(k,:)',...
                dFTtilde_opt_unsc(k,:)',full(lMTk_lr_opt_all),...
                full(vMTk_lr_opt_all),tensions,aTendon,shift);
        % Muscle fiber lengths and velocities at optimal solution
        [lM_opt_all_a,~] = f_FiberLength_TendonForce_tendon(...
            FTtilde_opt_unsc(k,:)',full(lMTk_lr_opt_all),aTendon,shift);                
        [vM_opt_all_a,~] = ...
            f_FiberVelocity_TendonForce_tendon(FTtilde_opt_unsc(k,:)',...
            dFTtilde_opt_unsc(k,:)',full(lMTk_lr_opt_all),...
            full(vMTk_lr_opt_all),aTendon,shift);
        FT_opt_all_new(k,:) = full(FT_opt_all_a);
        Fce_opt_all_new(k,:) = full(Fce_opt_all_a);
        Fpass_opt_all_new(k,:) = full(Fpass_opt_all_a);
        Fiso_opt_all_new(k,:) = full(Fiso_opt_all_a);
        lM_opt_all_new(k,:) = full(lM_opt_all_a);
        vM_opt_all_new(k,:) = full(vM_opt_all_a);
        lMT_opt_all_new(k,:) = full(lMTk_lr_opt_all);
        vMT_opt_all_new(k,:) = full(vMTk_lr_opt_all);
                        
    end 
    
    
    %% Decompose optimal cost
if decomposeCost
    J_opt           = 0;
    E_cost          = 0;
    A_cost          = 0;
    Arm_cost        = 0;
    Qdotdot_cost    = 0;
    Pass_cost       = 0;
    GRF_cost        = 0;
    vA_cost         = 0;
    dFTtilde_cost   = 0;
    QdotdotArm_cost = 0;
    count           = 1;
    h_opt           = tf_opt/N;
    for k=1:N      
        % Get muscle-tendon lengths, velocities, moment arms
        % Left leg
        qin_l_opt_all = [Xk_Qs_Qdots_opt(k,jointi.hip_flex.l*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.hip_add.l*2-1), ...
            Xk_Qs_Qdots_opt(k,jointi.hip_rot.l*2-1), ...
            Xk_Qs_Qdots_opt(k,jointi.knee.l*2-1), ...
            Xk_Qs_Qdots_opt(k,jointi.ankle.l*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.subt.l*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.ext*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.ben*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.rot*2-1)];  
        qdotin_l_opt_all = [Xk_Qs_Qdots_opt(k,jointi.hip_flex.l*2),...
            Xk_Qs_Qdots_opt(k,jointi.hip_add.l*2),...
            Xk_Qs_Qdots_opt(k,jointi.hip_rot.l*2),...
            Xk_Qs_Qdots_opt(k,jointi.knee.l*2),...
            Xk_Qs_Qdots_opt(k,jointi.ankle.l*2),...
            Xk_Qs_Qdots_opt(k,jointi.subt.l*2),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.ext*2),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.ben*2),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.rot*2)];  
        [lMTk_l_opt_all,vMTk_l_opt_all,~] = ...
            f_lMT_vMT_dM(qin_l_opt_all,qdotin_l_opt_all);    
        % Right leg
        qin_r_opt_all = [Xk_Qs_Qdots_opt(k,jointi.hip_flex.r*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.hip_add.r*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.hip_rot.r*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.knee.r*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.ankle.r*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.subt.r*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.ext*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.ben*2-1),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.rot*2-1)];  
        qdotin_r_opt_all = [Xk_Qs_Qdots_opt(k,jointi.hip_flex.r*2),...
            Xk_Qs_Qdots_opt(k,jointi.hip_add.r*2),...
            Xk_Qs_Qdots_opt(k,jointi.hip_rot.r*2),...
            Xk_Qs_Qdots_opt(k,jointi.knee.r*2),...
            Xk_Qs_Qdots_opt(k,jointi.ankle.r*2),...
            Xk_Qs_Qdots_opt(k,jointi.subt.r*2),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.ext*2),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.ben*2),...
            Xk_Qs_Qdots_opt(k,jointi.trunk.rot*2)];      
        [lMTk_r_opt_all,vMTk_r_opt_all,~] = ...
            f_lMT_vMT_dM(qin_r_opt_all,qdotin_r_opt_all);
        % Both legs
        lMTk_lr_opt_all = ...
            [lMTk_l_opt_all([1:43,47:49],1);lMTk_r_opt_all(1:46,1)];
        vMTk_lr_opt_all = ...
            [vMTk_l_opt_all([1:43,47:49],1);vMTk_r_opt_all(1:46,1)];        
        % Metabolic energy rate
        [~,~,Fce_opt_all,Fpass_opt_all,Fiso_opt_all,vMmax_opt_all,...
            massM_opt_all] = f_forceEquilibrium_FtildeState_all_tendon_2(...
                a_opt_unsc(k,:)',FTtilde_opt_unsc(k,:)',...
                dFTtilde_opt_unsc(k,:)',full(lMTk_lr_opt_all),...
                full(vMTk_lr_opt_all),tensions,aTendon,shift);                  
        [~,lMtilde_opt_all] = f_FiberLength_TendonForce_tendon(...
            FTtilde_opt_unsc(k,:)',full(lMTk_lr_opt_all),aTendon,shift);                
        [vM_opt_all,~] = ...
            f_FiberVelocity_TendonForce_tendon(FTtilde_opt_unsc(k,:)',...
            dFTtilde_opt_unsc(k,:)',full(lMTk_lr_opt_all),...
            full(vMTk_lr_opt_all),aTendon,shift);   
        
        if mE == 0 % Bhargava et al. (2004)
            [e_tot_all,~,~,~,~,~] = fgetMetabolicEnergySmooth2004all_2(...
                a_opt_unsc(k,:)',a_opt_unsc(k,:)',full(lMtilde_opt_all),...
                full(vM_opt_all),full(Fce_opt_all)',full(Fpass_opt_all)',...
                full(massM_opt_all)',pctsts,full(Fiso_opt_all)',...
                MTparameters_m(1,:)',body_mass,10); 
        elseif mE == 1 % Umberger et al. (2003)
            % vMtilde defined for this model as vM/lMopt
            vMtildeUmbk_opt_all = full(vM_opt_all)./(MTparameters_m(2,:)');
            [e_tot_all,~,~,~,~] = fgetMetabolicEnergySmooth2003all_2(...
                a_opt_unsc(k,:)',a_opt_unsc(k,:)',full(lMtilde_opt_all),...
                vMtildeUmbk_opt_all,full(vM_opt_all),full(Fce_opt_all)',...
                full(massM_opt_all)',pctsts,full(vMmax_opt_all)',...
                full(Fiso_opt_all)',body_mass,10);                
        elseif mE == 2 % Umberger (2010)
            % vMtilde defined for this model as vM/lMopt
            vMtildeUmbk_opt_all = full(vM_opt_all)./(MTparameters_m(2,:)');
            [e_tot_all,~,~,~,~] = fgetMetabolicEnergySmooth2010all_2(...
                a_opt_unsc(k,:)',a_opt_unsc(k,:)',full(lMtilde_opt_all),...
                vMtildeUmbk_opt_all,full(vM_opt_all),full(Fce_opt_all)',...
                full(massM_opt_all)',pctsts,full(vMmax_opt_all)',...
                full(Fiso_opt_all)',body_mass,10);            
        elseif mE == 3 % Uchida et al. (2016)
            % vMtilde defined for this model as vM/lMopt
            vMtildeUmbk_opt_all = full(vM_opt_all)./(MTparameters_m(2,:)');
            [e_tot_all,~,~,~,~] = fgetMetabolicEnergySmooth2016all_2(...
                a_opt_unsc(k,:)',a_opt_unsc(k,:)',full(lMtilde_opt_all),...
                vMtildeUmbk_opt_all,full(vM_opt_all),full(Fce_opt_all)',...
                full(massM_opt_all)',pctsts,full(vMmax_opt_all)',...
                full(Fiso_opt_all)',body_mass,10);      
        elseif mE == 4 % Umberger (2010) treating muscle lengthening 
            % heat rate as Umberger et al. (2003)
            % vMtilde defined for this model as vM/lMopt
            vMtildeUmbk_opt_all = full(vM_opt_all)./(MTparameters_m(2,:)');
            [e_tot_all,~,~,~,~] = fgetMetabolicEnergySmooth2010all_hl_2(...
                a_opt_unsc(k,:)',a_opt_unsc(k,:)',full(lMtilde_opt_all),...
                vMtildeUmbk_opt_all,full(vM_opt_all),full(Fce_opt_all)',...
                full(massM_opt_all)',pctsts,full(vMmax_opt_all)',...
                full(Fiso_opt_all)',body_mass,10);  
        elseif mE == 5 % Umberger (2010) treating negative mechanical 
            % work as Umberger et al. (2003)
            % vMtilde defined for this model as vM/lMopt
            vMtildeUmbk_opt_all = full(vM_opt_all)./(MTparameters_m(2,:)');
            [e_tot_all,~,~,~,~] = fgetMetabolicEnergySmooth2010all_neg_2(...
                a_opt_unsc(k,:)',a_opt_unsc(k,:)',full(lMtilde_opt_all),...
                vMtildeUmbk_opt_all,full(vM_opt_all),full(Fce_opt_all)',...
                full(massM_opt_all)',pctsts,full(vMmax_opt_all)',...
                full(Fiso_opt_all)',body_mass,10);  
        end
        e_tot_opt_all = full(e_tot_all)';
        for j=1:d                     
  
            J_opt = J_opt + 1/(dist_trav_opt)*(...
                W.E*B(j+1)      *...
                    (f_J92exp(e_tot_opt_all,exp_E))/body_mass*h_opt + ...                   
                W.A*B(j+1)      *(f_J92(a_opt_ext_col(count,:)))*h_opt +...      
                W.Ak*B(j+1)     *...
                    (f_J21(qdotdot_opt(k,residuals_noarmsi)))*h_opt + ...                     
                W.passMom*B(j+1)*(f_J15(Tau_pass_opt_all(k,:)))*h_opt + ...                    
                W.u*B(j+1)      *(f_J92(vA_opt(k,:)))*h_opt + ...        
                W.u*B(j+1)      *(f_J92(dFTtilde_opt(k,:)))*h_opt);                                   
                E_cost = E_cost + W.E*B(j+1)*...
                    (f_J92exp(e_tot_opt_all,exp_E))/body_mass*h_opt;
                A_cost = A_cost + W.A*B(j+1)*...
                    (f_J92(a_opt_ext_col(count,:)))*h_opt;
                Qdotdot_cost = Qdotdot_cost + W.Ak*B(j+1)*...
                    (f_J21(qdotdot_opt(k,residuals_noarmsi)))*h_opt;
                Pass_cost = Pass_cost + W.passMom*B(j+1)*...
                    (f_J15(Tau_pass_opt_all(k,:)))*h_opt;
                vA_cost = vA_cost + W.u*B(j+1)*...
                    (f_J92(vA_opt(k,:)))*h_opt;
                dFTtilde_cost = dFTtilde_cost + W.u*B(j+1)*...
                    (f_J92(dFTtilde_opt(k,:)))*h_opt;
                QdotdotArm_cost = QdotdotArm_cost + W.u*B(j+1)*...
                    (f_J8(qdotdot_opt(k,armsi)))*h_opt;                
                count = count + 1;                  
        end
    end      
    J_optf = full(J_opt);     
    E_costf = full(E_cost);
    A_costf = full(A_cost);
    Qdotdot_costf = full(Qdotdot_cost);
    Pass_costf = full(Pass_cost);
    vA_costf = full(vA_cost);
    dFTtilde_costf = full(dFTtilde_cost);
    QdotdotArm_costf = full(QdotdotArm_cost);
    % assertCost should be 0 
    assertCost = J_optf - 1/(dist_trav_opt)*(E_costf+A_costf+...
        Qdotdot_costf+Pass_costf+vA_costf+dFTtilde_costf+...
        QdotdotArm_costf);    
end 


    %% Optimal cost and CPU time
    pathDiary = [pathresults,'/',namescript,'/D',savename];
    [CPU_IPOPT,CPU_NLP,~,Cost,~,~,~,~,OptSol] = readDiary(pathDiary);

    %% Save results       
    if saveResults              
        
        if (exist([pathresults,'/',namescript,...
                '/Results_',namescript,'.mat'],'file')==2) 
            load([pathresults,'/',namescript,'/Results_',namescript,'.mat']);
        else
            Results_pred.([namescript,'_c',num2str(ww)]) = ...
                struct('Qs_opt',[]);
        end
        
        Results_pred.([namescript,'_c',num2str(ww)]).Qs_opt = ...
            q_opt_unsc.deg;
        Results_pred.([namescript,'_c',num2str(ww)]).Acts_opt = ...
            a_opt_unsc;
        Results_pred.([namescript,'_c',num2str(ww)]).Ts_opt = ...
            Tauk_out;
        Results_pred.([namescript,'_c',num2str(ww)]).GRFs_opt = ...
            GRF_opt_unsc;
        Results_pred.([namescript,'_c',num2str(ww)]).CPU_IPOPT = ...
            CPU_IPOPT;
        Results_pred.([namescript,'_c',num2str(ww)]).CPU_NLP = ...
            CPU_NLP;
        Results_pred.([namescript,'_c',num2str(ww)]).StrideLength_opt = ...
            StrideLength_opt;
        Results_pred.([namescript,'_c',num2str(ww)]).stride_width_mean = ...
            stride_width_mean;
        Results_pred.([namescript,'_c',num2str(ww)]).stride_width_std = ...
            stride_width_std;
        Results_pred.([namescript,'_c',num2str(ww)]).calc_Orig_r = ...
            calc_Orig_r;
        Results_pred.([namescript,'_c',num2str(ww)]).calc_Orig_l = ...
            calc_Orig_l;
        Results_pred.([namescript,'_c',num2str(ww)]).Cost = Cost;
        Results_pred.([namescript,'_c',num2str(ww)]).OptSol = OptSol;
        Results_pred.([namescript,'_c',num2str(ww)]).Tps_opt = ...
            Tau_pass_opt_all;
        Results_pred.([namescript,'_c',num2str(ww)]).FTtils_opt = ...
            FTtilde_opt_unsc;
        Results_pred.([namescript,'_c',num2str(ww)]).Exts_opt = ...
            e_opt_unsc;
        Results_pred.([namescript,'_c',num2str(ww)]).FTs_opt = ...
            FT_opt_all_new;
        Results_pred.([namescript,'_c',num2str(ww)]).FCEs_opt = ...
            Fce_opt_all_new;
        Results_pred.([namescript,'_c',num2str(ww)]).FPass_opt = ...
            Fpass_opt_all_new;
        Results_pred.([namescript,'_c',num2str(ww)]).FIso_opt = ...
            Fiso_opt_all_new;
        Results_pred.([namescript,'_c',num2str(ww)]).FibLen_opt = ...
            lM_opt_all_new;
        Results_pred.([namescript,'_c',num2str(ww)]).FibVel_opt = ...
            vM_opt_all_new;
        Results_pred.([namescript,'_c',num2str(ww)]).MTLen_opt = ...
            lMT_opt_all_new;
        Results_pred.([namescript,'_c',num2str(ww)]).MTVel_opt = ...
            vMT_opt_all_new;
        Results_pred.([namescript,'_c',num2str(ww)]).colheaders.joints = ...
            joints;
        Results_pred.([namescript,'_c',num2str(ww)]).colheaders.GRF =...
            {'fore_aft_r','vertical_r','lateral_r','fore_aft_l','vertical_l','lateral_l'};

        Results_pred.([namescript,'_c',num2str(ww)]).colheaders.muscles = ...
                muscleNames;
        Results_pred.([namescript,'_c',num2str(ww)]).colheaders.Tpjnts =...
            {'hip_flexion_l','hip_adduction_l',...
    'hip_rotation_l','hip_flexion_r','hip_adduction_r','hip_rotation_r',...
    'knee_angle_l','knee_angle_r','ankle_angle_l','ankle_angle_r',...
    'subtalar_angle_l','subtalar_angle_r',...
    'mtp_angle_l','mtp_angle_r',...
    'lumbar_extension','lumbar_bending','lumbar_rotation'};
        Results_pred.([namescript,'_c',num2str(ww)]).muscles_param.params = MTparameters_m;
        Results_pred.([namescript,'_c',num2str(ww)]).muscles_param.aTendon = aTendon;
        Results_pred.([namescript,'_c',num2str(ww)]).muscles_param.shift = shift;
        Results_pred.([namescript,'_c',num2str(ww)]).muscles_param.tensions = tensions;
        Results_pred.([namescript,'_c',num2str(ww)]).config. ...
            (['Speed_',num2str(v_tgt_id*100)]). ...  
            (['W_MetabolicEnergyRate_',num2str(W.E)]). ...
            (['W_MuscleActivity_',num2str(W.A)]). ...            
            (['W_JointAcceleration_',num2str(W.Ak)]). ...
            (['W_PassiveTorque_',num2str(W.passMom)]). ...
            (['W_ArmExcitation_',num2str(W.ArmE)]). ...               
            (['Power_MetabolicEnergyRate_',num2str(exp_E)]). ...
            (['InitialGuessType_',num2str(IGsel)]). ...
            (['InitialGuessMode_',num2str(IGm)]). ...
            (['InitialGuessCase_',num2str(IGcase)]). ... 
            (['WeaknessHipActuators_',num2str(h_weak)]). ...
            (['WeaknessAnklePlantarflexors_',num2str(pf_weak)]). ...   
            (['MetabolicEnergyModel_',num2str(mE)]). ...            
            (['ContactModel_',num2str(cm)]). ...  
            (['Number_MeshIntervals_',num2str(N)]). ...
            (['MaximumContractionVelocity_',num2str(vMax_s)]). ...
            (['CoContraction_',num2str(coCont)]) = 1;
        Results_pred.([namescript,'_c',num2str(ww)]).subject = subject;
        Results_pred.([namescript,'_c',num2str(ww)]).walk_trial = ...
            nametrial_walk.IK;
        Results_pred.([namescript,'_c',num2str(ww)]).all_weakness = ...
            all_weakness;
        Results_pred.([namescript,'_c',num2str(ww)]).tf_opt = tf_opt;
        Results_pred.([namescript,'_c',num2str(ww)]).Qdots_opt = ...
            qdot_opt_unsc.deg;
                
        % Save data
        save([pathresults,'/',namescript,'/Results_',namescript,'.mat'],...
            'Results_pred');
        
        if writeIKmotion
        pathOpenSim = [pathRepo,'/OpenSim'];
        addpath(genpath(pathOpenSim));

        JointAngle.labels = {'time','pelvis_tilt','pelvis_list',...
        'pelvis_rotation','pelvis_tx','pelvis_ty','pelvis_tz',...
        'hip_flexion_l','hip_adduction_l','hip_rotation_l',...
        'hip_flexion_r','hip_adduction_r','hip_rotation_r',...
        'knee_angle_l','knee_angle_r','ankle_angle_l','ankle_angle_r',...
        'subtalar_angle_l','subtalar_angle_r',...
        'mtp_angle_l','mtp_angle_r',...
        'lumbar_extension','lumbar_bending','lumbar_rotation'};    
    
        % New gait cycle % Two gait cycles
        q_opt_GUI_GC_2 = zeros(N,1+nq.all);
        q_opt_GUI_GC_2(:,1) = tgrid(:,1:end-1)';
        q_opt_GUI_GC_2(:,2:end) = q_opt_unsc.deg;
        
        % Combine data joint angles and muscle activations
        JointAngleMuscleAct.data = [q_opt_GUI_GC_2,a_opt_unsc];
        % Get muscle labels
        muscleNamesAll = cell(1,NMuscle);
        muscleNamesAll = muscleNames;

        % Combine labels joint angles and muscle activations
        JointAngleMuscleAct.labels = JointAngle.labels;
        for i = 1:NMuscle
            JointAngleMuscleAct.labels{i+size(q_opt_GUI_GC_2,2)} = ...
                [muscleNamesAll{i},'/activation'];
        end               
        
        filenameJointAngles = [pathRepo,'/Results/',subject,'/',namescript,...
                '/IK_',namescript,'_c',num2str(ww),'.mot'];
        write_motionFile(JointAngleMuscleAct, filenameJointAngles)
        end
        
    end    
end
end
