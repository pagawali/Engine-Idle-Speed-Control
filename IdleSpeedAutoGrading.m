%% EE5812 Automotive Control Systems
%  Auto Grading: Idle Speed Control Project
%  3/1/2019

%% ----------------------------------------------------------------------------------- %%
%  ------ NOTE:  This script is for the Integral LQR Idle Speed Controller with integral spark error cost
% ------------------------------------------------------------------------------------ %%

%% Parameters
Fixed_step_size = 0.01;
N_d = 600;      % Engine speed (RPM)
Us_d = -20;     % Equilibrium spark timing (deg)
Ua_d = 0.06;    % Equilibrium throttle setting (mm)

% Specifications
Max_Np = 20;           % Desired positive engine speed tolerance [RPM]
Min_Nm =-30;           % Desired negative engine speed tolerance [RPM]
Time_settling_d = 1;   % Desired Settling time [sec]
Max_OSi = 6;           % Desired overshoot for increasing load.
Max_OSd = 1;           % Desired overshoot for decreasing load.
Max_E_ssN_d = 0.01;    % Desired steady-state error [RPM]
Max_Ua = 0.12;         % Desired tolerance of the throttle input
Min_Ua = 0;            % Desired tolerance of the throttle input
Max_Us = 0;            % Desired tolerance of the sparking timing
Min_Us = -40;          % Desired tolerance of the sparking timing
Max_Us_Trans = -10;    % Demonstrate that sparking timing is being used
Max_E_ssUs_d = 0.01;   % Desired steady-state spark timing error [degrees]

T_S_R = 0.02*40;
lbl = 'With Integral Control';

Settlingtime_thres = [N_d-T_S_R, N_d+T_S_R]; % Settling Time Thresholds


%%
if exist('Us','var') == 1 && exist('Ua','var') == 1 && exist('N','var') == 1  % Check if variables Us Ua and N exist (if YES)
    % Overshoot Calculation
      % For increasing load
      N_i = N(19/Fixed_step_size:29/Fixed_step_size,2);
      OSi = (max(N_i)-600);   % Measured overshoot of N - increasing load
      % For decreasing step
      N_dd = N(39/Fixed_step_size:49/Fixed_step_size,2);
      OSd = (600-min(N_dd));  % Measured overshoot of N - decreasing load
    % Settling Time Calculation
    Cntr = 1;
    for aa = 40/Fixed_step_size:59/Fixed_step_size
         if (N(aa,2) <= Settlingtime_thres(1)) || (N(aa,2) >= Settlingtime_thres(2))
             Settling_Data_Set(Cntr,:) =  N(aa,:);
             Cntr = Cntr + 1;
         end
    end

    Time_Settling_m = max(Settling_Data_Set(:,1))-40;
    % Steady-state Error Calculation
    SS_Error = max(N(59/Fixed_step_size-200:59/Fixed_step_size,2))-max(N(39/Fixed_step_size-200:39/Fixed_step_size,2));
    % Throttle Input Calculation
    Diff_Throttle = max(Ua(20.1/Fixed_step_size-200:39/Fixed_step_size,2))-Ua(39/Fixed_step_size,2);
    % Spark Timing Calculation
    Spark = Us(:,2);

    %% Performances Checking
    %  RPM Transient Checking 
        RPM_e_max = (max(N(10/Fixed_step_size:end,2))-N_d);
        RPM_e_min = (min(N(10/Fixed_step_size:end,2))-N_d);
        if RPM_e_max <= Max_Np               % check if RPM stays less than 600+20
            % Full credits
            Comment_RPMmax = ['FullCredit: Maximum RPM error is less than ',num2str(Max_Np),'[RPM]'];
        elseif RPM_e_max <= 1.5*Max_Np
            % Half credits
            Comment_RPMmax = ['HalfCredit: Maximum RPM error is greater than ', num2str(Max_Np),' [RPM] but less than ', num2str(1.5*Max_Np),' [RPM]'];
        else
            % No credits
            Comment_RPMmax = ['NoCredit: Maximum RPM error is more than ',num2str(1.5*Max_Np), ' [RPM]'];
        end
        if RPM_e_min >= Min_Nm                % check if RPM stays greater than 600-30
            % Full credits
            Comment_RPMmin = ['FullCredit: Minimum RPM error is greater than ',num2str(Min_Nm),'[RPM]'];
        elseif RPM_e_min >= 1.5*Min_Nm
            % Half credits
            Comment_RPMmin = ['HalfCredit: Minimum RPM error is greater than ', num2str(1.5*Min_Nm),' [RPM] but less than ', num2str(Min_Nm),' [RPM]'];
        else
            % No credits
            Comment_RPMmin = ['NoCredit: Minimum RPM is less than ',num2str(1.5*Min_Nm), ' [RPM]'];
        end
    %  2% Settling Time Checking
    if Time_Settling_m <= Time_settling_d       % check if setlling time is less than 0.8
        % Full credits
        Comment_st = ['FullCredit: Settling Time is less than ', num2str(Time_settling_d),' [Sec]'];
    elseif Time_Settling_m <= 1.5*Time_settling_d
        % Half credits
        Comment_st = ['HalfCredit: Settling Time is more than ', num2str(Time_settling_d), ' [Sec] but less than ', num2str(1.5*Time_settling_d),' [Sec]'];
    else
        % No credits
        Comment_st = ['NoCredit: Settling Time is greater than ',num2str(1.5*Time_settling_d),' [Sec]'];
    end
    %  Steady-state Error Checking
    if abs(SS_Error) <= Max_E_ssN_d
        % Full credits
        Comment_ss = ['FullCredit: The steady-state value is less than ', num2str(Max_E_ssN_d)];
    elseif abs(SS_Error) <= 1.5*Max_E_ssN_d
        % Half credits
        Comment_ss = ['HalfCredit: The steady-state value is more than ', num2str(Max_E_ssN_d),' but less than ', num2str(1.5*Max_E_ssN_d)];
    else
        % No credits
        Comment_ss = ['NoCredit: The steady-state value is more than ',num2str(1.5*Max_E_ssN_d)];
    end
    % % Overshoot check - increasing load.
    if OSi <= Max_OSi
        % Full credits
        Comment_OSi = ['FullCredit: The %Overshoot (increasing load) is less than ', num2str(Max_OSi)];
    elseif abs(SS_Error) <= 1.5*Max_E_ssN_d
        % Half credits
        Comment_OSi = ['HalfCredit: The %Overshoot (increasing load) is more than ', num2str(Max_OSi),' but less than ', num2str(1.5*Max_OSi)];
    else
        % No credits
        Comment_OSi = ['NoCredit: The %Overshoot (increasing load) is more than ',num2str(1.5*Max_OSi)];
    end
    % % Overshoot check - decreasing load.
    if OSd <= Max_OSd
        % Full credit
        Comment_OSd = ['FullCredit: The Overshoot (decreasing load) is less than ', num2str(Max_OSi)];
    elseif abs(SS_Error) <= 1.5*Max_E_ssN_d
        % Half credit
        Comment_OSd = ['HalfCredit: The Overshoot (decreasing load) is more than ', num2str(Max_OSi),' but less than ', num2str(1.5*Max_OSi)];
    else
        % No credit
        Comment_OSd = ['NoCredit: The %Overshoot (decreasing load) is more than ',num2str(1.5*Max_OSi)];
    end
    %  Throttle Input Checking
    if max(Ua(1000:8000,2)) <= Max_Ua && Min_Ua <= min(Ua(1000:8000,2))       % check if throttle difference is in between 0 and  0.15
        % Full credit
        Comment_ti = ['FullCredit: The Throttle Input remains in [', num2str(Min_Ua),' ', num2str(Max_Ua), ' ]'];
    else
        % No credit
        Comment_ti = ['NoCredit: The Throttle Input goes outside of ',num2str(1.5*Max_Ua),' of the nominal value'];
    end
    %  Spark Timing Limits Checking
    if max(Spark(1000:8001)) <= Max_Us && Min_Us <= min(Spark(1000:8001))   % check if spark stays between 0 and -40
        % Full credit
        Comment_spt = ['FullCredit: The Spark Timing remains in [',num2str(Max_Us), ' ', num2str(Min_Us),'] degrees'];
    else
        % No credit
        Comment_spt = ['NoCredit: The Spark Timing goes out of [',num2str(Max_Us),num2str(Max_Us),'] deg.'];
    end
    %  Spark Timing Usage Checking
    if max(Spark(1000:8001)) >= Max_Us_Trans   % check if spark goes above -10
        % Full credit
        Comment_sptm = ['FullCredit: The Spark Timing maximum is above ', num2str(Max_Us_Trans),' deg.'];
    elseif max(Spark(1000:8001)) <= 0.7*Max_Us_Trans
        % Half credit
        Comment_sptm = ['HalfCredit: The Spark Timing maximum is in the range: ', num2str(0.7*Max_Us_Trans), num2str(Max_Us_Trans),' degrees'];
    else
        % No credits
        Comment_sptm = ['NoCredit: The Spark Timing maximum is below ', num2str(0.7*Max_Us_Trans),' deg.'];
    end
    
    %% Plotting
    figure(1);
    plot(N(:,1),N(:,2)); grid minor;
    xlabel('Time [Sec]');
    ylabel('Engine Speed [RPM]');
    title(['Engine Speed vs. Time ',lbl]);
        tx1 = text(5,620,Comment_RPMmax);
        tx2 = text(5,615,Comment_RPMmin);
        tx3 = text(5,610,Comment_st);
        tx4 = text(5,590,Comment_ss);
        tx5 = text(5,585,Comment_OSi);
        tx6 = text(5,580,Comment_OSd);
    
    
    % savefig(figure(1),['N ', lbl,'.fig']);
    
    figure(2);
    plot(Ua(:,1),Ua(:,2)); grid minor;
    xlabel('Time [Sec]');
    ylabel('Throttle Input [mm]');
    title(['Throttle Input U_a vs. Time ',lbl]);
        tx7 = text(5,0.04,Comment_ti);
    % savefig(figure(2),['Ua ', lbl,'.fig']);
   
    figure(3);
    plot(Us(:,1),Us(:,2)); grid minor;
    xlabel('Time [Sec]');
    ylabel('Spark Timing [deg]');
    title(['Spark Timing U_s vs. Time ',lbl]);
        tx8 = text(5,-25,Comment_spt);  
        tx9 = text(5,-29,Comment_sptm); 
    % savefig(figure(3),['Us ', lbl,'.fig']);
    
    
else        % Check if variables Us Ua and N exist (if NOT)
	msgbox('The Variables [Us] [Ua] and [N] are MISSING! Please Check the Variable Names of the Scopes in LOGGING Setting Menu in Simulink!','Comments');
end
disp(['Maximum RPM = ', num2str(RPM_e_max)])
disp(Comment_RPMmax)
disp(' ')
disp(['Minimum RPM = ', num2str(RPM_e_min)])
disp(Comment_RPMmin)
disp(' ')
disp(['Settling time (2%) = ', num2str(Time_Settling_m)])
disp(Comment_st)
disp(' ')
disp(['Steady-state error = ', num2str(SS_Error)])
disp(Comment_ss)
disp(' ')
disp(['Overshoot (increasing load) = ', num2str(OSi)])
disp(Comment_OSi)
disp(' ')
disp(['Overshoot (decreasing load) = ', num2str(OSd)])
disp(Comment_OSd)
disp(' ')
disp(['Range of throttle inputs goes from ', num2str(min(Ua(:,2))), ' to ' num2str(max(Ua(:,2)))])
disp(Comment_ti)
disp(' ')
disp(['Range of spark timing goes from ', num2str(min(Spark(1000:8001))), ' to ' num2str(max(Spark(1000:8001)))])
disp(Comment_spt)
disp(' ')
disp(['Maximum spark timing = ', num2str(max(Spark(1000:8001)))])
disp(Comment_sptm)

