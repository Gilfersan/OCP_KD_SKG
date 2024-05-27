%%  Muscle-tendon parameter optimal estimation
% This script estimates optimal fiber lengths and tendon slack lengths
% while solving the muscle rendundancy problem simultaneously for gait
% trials and passive stretches. The problem is solved via direct
% collocation. Details of the problem formualtion are in the associated
% publication.
%
% The code is not perfectly suited for use with data from different
% subjects although only minimal adjustment should be made. The code is
% also pretty messy and likely difficult to use with different data
% structure.
%
% Author: Antoine Falisse
% Date: 1/2/2020.
% Edited by Gilmar Fernandes dos Santos on 07.11.2021 
clear all
close all
clc

p_new = mfilename('fullpath');
[folderscript,~,~] = fileparts(p_new);
cd(folderscript)

idx_ww = [4]; % Index row in matrix settings

%% Select settings
for www = 1:length(idx_ww)
ww = idx_ww(www);

pathMain = pwd;
[pathRepo_2,~,~] = fileparts(pathMain);
[pathRepo,~,~] = fileparts(pathRepo_2);
addpath(genpath(pathRepo));

Settings_ParamEstim

%% User settings
solveProblem = 1;   % set to 1 to solve problem
saveResults = 1;    % set to 1 to save results
saveParameters = 1; % set to 1 to save MT-parameters
showResults = 0;    % set to 1 to plot results

%% Problem settings
subject = 'subjectKD'; 
subject_id = 'KD';
subject_cond = 'NRh';


lMtilde_ext.max = 1.5;  % reference maximum normalized fiber length
lMtilde_ext.min = 0.5;  % reference minimum normalized fiber length

NMeshes = settings(ww,6); % number of mesh intervals
NGaitTrials = '5'; % number of gait trials

sides_num = settings(ww,7);

if sides_num == 1
sides = {'l','r'};
elseif sides_num == 2
sides = {'r'};    
elseif sides_num == 3
sides = {'l'};    
end

NSides = num2str(length(sides));

% Weight factors in cost function
W.a         = settings(ww,1); % muscle activations
W.aT        = settings(ww,2); % reserve actuators
W.vA        = settings(ww,3); % time derivative of muscle activations
W.dF        = settings(ww,4); % time derivative of muscle forces
W.lMopt     = settings(ww,5); % optimal fiber lengths (short fibers)

%% Helper variables / paths
% Paths

pppp = mfilename('fullpath');
[~,namescript,~] = fileparts(pppp);
pathOpenSimModel = [pathRepo,'/OpenSimModel'];
pathCollocationScheme = [pathRepo,'/CollocationScheme'];
addpath(genpath(pathCollocationScheme));
pathResults = [pathRepo,'/ParameterEstimation/Results/',subject,'/',namescript,'/'];
pathMuscleModel = [pathRepo,'/MuscleModel/'];
addpath(genpath(pathMuscleModel));
pathVariousFunctions = [pathRepo,'/VariousFunctions'];
addpath(genpath(pathVariousFunctions));
pathFun = [pathRepo,'/ParameterEstimation/functions_PaEs_ocp/'];
addpath(genpath(pathFun));

W.all = [W.a,W.aT,W.vA,W.dF,W.lMopt];
if round(sum(W.all),10)~=1
    disp('WARNING: the sum of the weights is not 1');
end


%% Solve optimal control problem
if solveProblem
% Degrees of freedom taken into account when solving the muscle redundancy
% problem for gait trials
gaitDOFNames = {'hip_flexion_','hip_adduction_','hip_rotation_','knee_angle_',...
    'ankle_angle_','subtalar_angle_'};
% Muscles
MuscleNames = {'glut_med1_','glut_med2_','glut_med3_',...
        'glut_min1_','glut_min2_','glut_min3_','semimem_',...
        'semiten_','bifemlh_','bifemsh_','sar_','add_long_',...
        'add_brev_','add_mag1_','add_mag2_','add_mag3_','tfl_',...
        'pect_','grac_','glut_max1_','glut_max2_','glut_max3_',......
        'iliacus_','psoas_','quad_fem_','gem_','peri_',...
        'rect_fem_','vas_med_','vas_int_','vas_lat_','med_gas_',...
        'lat_gas_','soleus_','tib_post_','flex_dig_','flex_hal_',...
        'tib_ant_','per_brev_','per_long_','per_tert_','ext_dig_',...
        'ext_hal_','ercspn_','intobl_','extobl_'};
% Muscles for which we optimize optimal fiber lengths and tendon slack
% lengths
MuscleNames_opt = {'glut_med1_','glut_med2_','glut_med3_',...
        'glut_min1_','glut_min2_','glut_min3_','semimem_',...
        'semiten_','bifemlh_','bifemsh_','sar_','add_long_',...
        'add_brev_','add_mag1_','add_mag2_','add_mag3_','tfl_',...
        'pect_','grac_','glut_max1_','glut_max2_','glut_max3_',......
        'iliacus_','psoas_','quad_fem_','gem_','peri_',...
        'rect_fem_','vas_med_','vas_int_','vas_lat_','med_gas_',...
        'lat_gas_','soleus_','tib_post_','flex_dig_','flex_hal_',...
        'tib_ant_','per_brev_','per_long_','per_tert_','ext_dig_',...
        'ext_hal_'};

if sides_num == 2 %right side

gaitDOFNames = {'hip_flexion_','hip_adduction_','hip_rotation_'};
% Muscles
MuscleNames = {'glut_med1_','glut_med2_','glut_med3_',...
        'glut_min1_','glut_min2_','glut_min3_','semimem_',...
        'semiten_','bifemlh_','sar_','add_long_',...
        'add_brev_','add_mag1_','add_mag2_','add_mag3_','tfl_',...
        'pect_','grac_','glut_max1_','glut_max2_','glut_max3_',......
        'iliacus_','psoas_','quad_fem_','gem_','peri_',...
        'rect_fem_','ercspn_','intobl_','extobl_'};
% Muscles for which we optimize optimal fiber lengths and tendon slack
% lengths
MuscleNames_opt = {'glut_med1_','glut_med2_','glut_med3_',...
        'glut_min1_','glut_min2_','glut_min3_','semimem_',...
        'semiten_','bifemlh_','sar_','add_long_',...
        'add_brev_','add_mag1_','add_mag2_','add_mag3_','tfl_',...
        'pect_','grac_','glut_max1_','glut_max2_','glut_max3_',......
        'iliacus_','psoas_','quad_fem_','gem_','peri_',...
        'rect_fem_'};    
    
    
end
    
    
% Load parameters of FLV relationships
load([pathMuscleModel,'Faparam.mat']);
load([pathMuscleModel,'Fpparam.mat']);
load([pathMuscleModel,'Fvparam.mat']);

%% Extract and arrange data
NTr = 0;
for i = 1:length(sides)
    side_tr = sides{i};
    
          
    switch subject
        case 'subjectKD'
            
            switch subject_cond
            case 'Rheo'
            % Available trials in database. Only trials used in results of
            % publication are shared.
            gait_trials_id = {'10','10','13','13','14','14','15','15','16','16'};
            gait_trials_leg = {'r','l','r','l','r','l','r','l','r','l'};
            gait_trials_start_time = [4.85,4.70,4.93,4.79,4.88,4.74,5.00,4.86,4.765,4.63];            
            gait_trials_end_time = [5.85,5.71,5.94,5.81,5.885,5.755,6.01,5.875,5.775,5.635];         
            % Select trials               
            trials_number = {'10','13','14','15','16'};
            case 'NRh'
            % Available trials in database. Only trials used in results of
            % publication are shared.
            gait_trials_id = {'17','17','19','19','20','20','21','21','22','22'};
            gait_trials_leg = {'r','l','r','l','r','l','r','l','r','l'};
            gait_trials_start_time = [4.58,4.445,4.57,4.435,4.68,4.545,5.345,5.205,4.645,4.515];            
            gait_trials_end_time = [5.565,5.435,5.57,5.435,5.66,5.535,6.33,6.195,5.63,5.51];         
            % Select trials                     
            trials_number = {'17','19','20','21','22'}; 
            
            end
                       
            
            count = 1;
            for tt = 1:length(gait_trials_leg)
                if strcmp(gait_trials_leg{tt},side_tr)
                    idx_leg(count) = tt;                        
                    count = count + 1;
                end
            end 
            gait_trials_start_time_sel = gait_trials_start_time(idx_leg);
            gait_trials_end_time_sel = gait_trials_end_time(idx_leg);
            gait_trials_id_sel = gait_trials_id(idx_leg);  
            count = 1;
            for tt = 1:length(trials_number)
                trials{tt} = [trials_number{tt}];   
                if find(strcmp(trials_number{tt},gait_trials_id_sel))
                    idx_leg_tr(count) = ...
                        find(strcmp(trials_number{tt},gait_trials_id_sel));                        
                    count = count + 1;
                end
            end  
            time(1,:) = gait_trials_start_time_sel(idx_leg_tr);
            time(2,:) = gait_trials_end_time_sel(idx_leg_tr)-0.05; 
            
    end 
    
   % Load linearly-scaled MT-parameters
    load([pathMuscleModel,'MTparameters_',subject,'.mat']);
    MTparameters_m = [MTparameters];
    NMuscles = size(MTparameters_m,2);
      
   %% Extract data from gait trials
   
   for j = 1:length(trials)
       trial = trials{j};
       trial_number = trials_number{j};
       trial_number_aux = str2num(trial_number);
       leg = side_tr;
       leg_up = 'L';
       if strcmp(leg,'r')
           leg_up = 'R';
       end
       side_t{j+NTr} = sides{i};
%        Fmax{j+NTr} = MTparameters_m(1,1:NMuscles/2);
       ParametersInit.(sides{i}).Fmax = MTparameters_m(1,1:46);
       ParametersInit.(sides{i}).OptFibLen = MTparameters_m(2,1:46);
       ParametersInit.(sides{i}).TenSlackLen = MTparameters_m(3,1:46);
       ParametersInit.(sides{i}).PennAng = MTparameters_m(4,1:46);
       if strcmp(leg,'r')
           ParametersInit.(sides{i}).Fmax = MTparameters_m(1,47:NMuscles);
           ParametersInit.(sides{i}).OptFibLen = MTparameters_m(2,47:NMuscles);
           ParametersInit.(sides{i}).TenSlackLen = MTparameters_m(3,47:NMuscles);
           ParametersInit.(sides{i}).PennAng = MTparameters_m(4,47:NMuscles);
       end
       % get ID
       pathID = [pathOpenSimModel,'/',subject,'/ID/inverse_dynamics_',subject_id,trial,...
           '.sto'];
       
       ID = importdata(pathID);
       for k = 1:length(gaitDOFNames)
           JointMom{j+NTr}(:,k) = ...
               ID.data(:,strcmp(ID.colheaders,[gaitDOFNames{k},leg,'_moment']));           
       end
       % select interval and interpolate
       step = (round(time(2,j),2) - round(time(1,j),2))/(NMeshes-1);
       interval = round(time(1,j),2):step:round(time(2,j),2);
       timeOpt{j+NTr}(1) = interval(1); timeOpt{j+NTr}(2) = interval(end); 
       JointMom{j+NTr} = interp1(ID.data(:,1),JointMom{j+NTr},interval);      
       % get lMT
       lMTpath = [pathOpenSimModel,'/',subject,'/MuscleAnalysis/',subject_id,...
           trial,'/',subject_id,trial,'_MuscleAnalysis_Length.sto'];
       lMT = importdata(lMTpath);
       for k = 1:length(MuscleNames)
           MuscTenLen{j+NTr}(:,k) = ...
               lMT.data(:,strcmp(lMT.colheaders,[MuscleNames{k},leg]));      
       end 
       % select interval and interpolate
       MuscTenLen{j+NTr} = interp1(lMT.data(:,1),MuscTenLen{j+NTr},interval); 
       % compute muscle tendon velocities (vMT)
       for m = 1:length(MuscleNames)
           pp_y = spline(interval,MuscTenLen{j+NTr}(:,m));
           [~,MuscTenVel{j+NTr}(:,m),~] = SplineEval_ppuval(pp_y,interval,1);
       end      
       % get MA
       MApath = [pathOpenSimModel,'/',subject,'/MuscleAnalysis/',subject_id,...
           trial,'/',subject_id,trial,'_MuscleAnalysis_MomentArm_'];
       for d = 1:length(gaitDOFNames)
           MApathDOF = [MApath,gaitDOFNames{d},leg,'.sto'];
           MA = importdata(MApathDOF);           
           for k = 1:length(MuscleNames)
               MuscMomArm{j+NTr}(:,k,d) = ...
                   MA.data(:,strcmp(MA.colheaders,[MuscleNames{k},leg])); 
           end           
       end
       idx_spanning{j+NTr} = sum(squeeze(sum(MuscMomArm{j+NTr},1))',1);
       idx_spanning{j+NTr}(idx_spanning{j+NTr}<=0.0001 & ...
           idx_spanning{j+NTr}>=-0.0001) = 0;
       idx_spanning{j+NTr}(idx_spanning{j+NTr}~=0) = 1;
%        Fmax{j+NTr} = Fmax{j+NTr}(idx_spanning{j+NTr}==1);
       % select interval and interpolate
       MuscMomArm{j+NTr} = interp1(MA.data(:,1),MuscMomArm{j+NTr},interval);
      
       % get muscleNames       
       for k = 1:length(MuscleNames)
        muscleNames.(leg).s{k} = [MuscleNames{k},leg];
       end
       MuscleNames_Spanning = MuscleNames(idx_spanning{j+NTr}==1);
       % get Side
       Side = leg_up;
       % Helper vector
       % SelectedMusclesInfo contains the indices of the muscles selected
       % for the optimization in the vector with all the muscles
        count = 1;
        for ii = 1:length(MuscleNames)
            if find(strcmp(MuscleNames_opt,MuscleNames{ii}))
                SelectedMusclesInfo{j+NTr}.Bool(ii) = true;       
                SelectedMusclesInfo{j+NTr}.Index(count) = ii;
                count = count+1;
            else
                SelectedMusclesInfo{j+NTr}.Bool(ii) = false; 
            end
        end
       % Helper vector
       % SelectedMusclesInfoSelSpanning contains the indices of the muscles
       % spanning the degrees of freedom of that trial in the vector of
       % muscles selected for the optimization
        count = 1;
        for ii = 1:length(MuscleNames_opt)
            if find(strcmp(MuscleNames_Spanning,MuscleNames_opt{ii}))
                SelectedMusclesInfoSelSpanning{j+NTr}.Bool(ii) = true;       
                SelectedMusclesInfoSelSpanning{j+NTr}.Index(count) = ii;
                count = count+1;
            else
                SelectedMusclesInfoSelSpanning{j+NTr}.Bool(ii) = false; 
            end
        end     
        % Helper vector
        % SelectedMusclesInfoSpanningSel contains the indices of the
        % muscles selected for the optimization in the vector of
        % muscles spanning the degrees of freedom
        count = 1;
        for ii = 1:length(MuscleNames_Spanning)
            if find(strcmp(MuscleNames_opt,MuscleNames_Spanning{ii}))
                SelectedMusclesInfoSpanningSel{j+NTr}.Bool(ii) = true;       
                SelectedMusclesInfoSpanningSel{j+NTr}.Index(count) = ii;
                count = count+1;
            else
                SelectedMusclesInfoSpanningSel{j+NTr}.Bool(ii) = false; 
            end
        end 
        % Helper vector
        % SelectedMusclesInfoOptSpanning contains the indices of the muscles
        % selected for the optimization and spanning the degrees of freedom
        count = 1;
        for ii = 1:length(MuscleNames)
            if (find(strcmp(MuscleNames_opt,MuscleNames{ii}))) ...
                    & (find(strcmp(MuscleNames_Spanning,MuscleNames{ii})))
                SelectedMusclesInfoOptSpanning{j+NTr}.Bool(ii) = true;       
                SelectedMusclesInfoOptSpanning{j+NTr}.Index(count) = ii;
                count = count+1;
            else
                SelectedMusclesInfoOptSpanning{j+NTr}.Bool(ii) = false; 
            end
        end        
    end
    
    NTr = length(trials);
   
end 
    
%% Solve optimal control problem and estimate muscle-tendon parameters
if ~(exist(pathResults,'dir')==7)
        mkdir(pathResults);
end
if (exist([pathResults,'/D_',namescript,'_c',num2str(ww)],'file')==2)
        delete ([pathResults,'/D_',namescript,'_c',num2str(ww)])
end 
diary([pathResults,'/D_',namescript,'_c',num2str(ww)]);
[params,activations,residuals,forces,lMtilde,time,stats,extlMT] = ...
    PaEs_ocp_Gil16(JointMom,MuscMomArm,...
    ParametersInit,MuscTenLen,MuscTenVel,SelectedMusclesInfo,...
    muscleNames,sides,Fvparam,Fpparam,Faparam,lMtilde_ext,W,...
    idx_spanning,...
    SelectedMusclesInfoSelSpanning,SelectedMusclesInfoOptSpanning,...
    SelectedMusclesInfoSpanningSel,timeOpt,side_t);
diary off
for i = 1:length(sides)
    side_tr = sides{i};        
    % Re-arrange parameters to includes the ones that were not optimized
    PARAMETERSall.(sides{i}).PennAng = ParametersInit.(sides{i}).PennAng;    
    PARAMETERSall.(sides{i}).OptFibLen = params.(sides{i}).OptFibLen;
    PARAMETERSall.(sides{i}).TenSlackLen = params.(sides{i}).TenSlackLen;
    PARAMETERSall.(sides{i}).MaxIsomForce = params.(sides{i}).Fmax;
    PARAMETERSall.(sides{i}).MaxContVelo = ...
        10*PARAMETERSall.(sides{i}).OptFibLen; 
    ParameterEstimationResults.PARAMETERS.(sides{i}) = PARAMETERSall.(sides{i});             
    ParametersInit.(sides{i}).MaxContVelo = ...
        10*ParametersInit.(sides{i}).OptFibLen;
    
    ParameterEstimationResults.ParametersInit.(sides{i}) = ...
        ParametersInit.(sides{i});
     
    
end
ParameterEstimationResults.extlMT = extlMT;
 
ParameterEstimationResults.Activations = activations;
ParameterEstimationResults.Forces = forces;
ParameterEstimationResults.Residuals = residuals;   
ParameterEstimationResults.lMtilde = lMtilde;
ParameterEstimationResults.Time = time;
ParameterEstimationResults.idx_spanning = idx_spanning;
ParameterEstimationResults.stats = stats;    
if saveResults
    if ~(exist(pathResults,'dir')==7)
        mkdir(pathResults);
    end
    save([pathResults,'/ParameterEstimationResults_',namescript,'_c',num2str(ww)],....
        'ParameterEstimationResults');
end
end

%% Save parameters
clear MTparameters
if saveParameters
% load MT-parameters
load([pathResults,'/ParameterEstimationResults_',namescript,'_c',num2str(ww)]);

if sides_num == 1

% Left side
MTparameters(1,1:46) = ParameterEstimationResults.PARAMETERS.l.MaxIsomForce;
MTparameters(2,1:46) = ParameterEstimationResults.PARAMETERS.l.OptFibLen;
MTparameters(3,1:46) = ParameterEstimationResults.PARAMETERS.l.TenSlackLen;
MTparameters(4,1:46) = ParameterEstimationResults.PARAMETERS.l.PennAng;
MTparameters(5,1:46) = ParameterEstimationResults.PARAMETERS.l.MaxContVelo;
% Right side
MTparameters(1,47:76) = ParameterEstimationResults.PARAMETERS.r.MaxIsomForce;
MTparameters(2,47:76) = ParameterEstimationResults.PARAMETERS.r.OptFibLen;
MTparameters(3,47:76) = ParameterEstimationResults.PARAMETERS.r.TenSlackLen;
MTparameters(4,47:76) = ParameterEstimationResults.PARAMETERS.r.PennAng;
MTparameters(5,47:76) = ParameterEstimationResults.PARAMETERS.r.MaxContVelo;

elseif sides_num == 2
sides = {'r'};    

% Left side
MTparameters(1,1:46) = MTparameters_m(1,1:46);
MTparameters(2,1:46) = MTparameters_m(2,1:46);
MTparameters(3,1:46) = MTparameters_m(3,1:46);
MTparameters(4,1:46) = MTparameters_m(4,1:46);
MTparameters(5,1:46) = MTparameters_m(5,1:46);
% Right side
MTparameters(1,47:76) = ParameterEstimationResults.PARAMETERS.r.MaxIsomForce;
MTparameters(2,47:76) = ParameterEstimationResults.PARAMETERS.r.OptFibLen;
MTparameters(3,47:76) = ParameterEstimationResults.PARAMETERS.r.TenSlackLen;
MTparameters(4,47:76) = ParameterEstimationResults.PARAMETERS.r.PennAng;
MTparameters(5,47:76) = ParameterEstimationResults.PARAMETERS.r.MaxContVelo;

elseif sides_num == 3
sides = {'l'};    

% Left side
MTparameters(1,1:46) = ParameterEstimationResults.PARAMETERS.l.MaxIsomForce;
MTparameters(2,1:46) = ParameterEstimationResults.PARAMETERS.l.OptFibLen;
MTparameters(3,1:46) = ParameterEstimationResults.PARAMETERS.l.TenSlackLen;
MTparameters(4,1:46) = ParameterEstimationResults.PARAMETERS.l.PennAng;
MTparameters(5,1:46) = ParameterEstimationResults.PARAMETERS.l.MaxContVelo;
% Right side
MTparameters(1,47:76) = MTparameters_m(1,47:NMuscles);
MTparameters(2,47:76) = MTparameters_m(2,47:NMuscles);
MTparameters(3,47:76) = MTparameters_m(3,47:NMuscles);
MTparameters(4,47:76) = MTparameters_m(4,47:NMuscles);
MTparameters(5,47:76) = MTparameters_m(5,47:NMuscles);

end


% save parameters
save([pathResults,'/MTparameters_mod_',namescript,'_c',num2str(ww),'.mat'],'MTparameters');
end

clearvars -except idx_ww
end
