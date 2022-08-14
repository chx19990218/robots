%% NMPC based Quadrotor trajectory tracking
clear all
close all
clc
%% define NMPC problem
NMPC_problem_formulation;
load NMPC_problem_definition.mat
%% control simulation

% connect to vrep
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);      % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1' ,19997,true,true,5000,5); % connect to vrep server
if (clientID>-1)
    disp('Connected to remote API server');
    
    % timestep
    dt = 0.01;
    vrep.simxSetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step,dt,vrep.simx_opmode_oneshot_wait);
    % set sychronous mode
    vrep.simxSynchronous(clientID,true);
    % start the simulation:
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait);
    % trigger one step simulation
%     vrep.simxSynchronousTrigger(id);
%     handles = Quadrotor_workcell_init(vrep,id);
%     Quadrotor = handles.Quadrotor;
    pose0 = zeros(6,1);
    velocity0 = zeros(6,1);
    [res,Bill_base] = vrep.simxGetObjectHandle(clientID,'Bill_base',vrep.simx_opmode_blocking);
    [res,Quadricopter_base] = vrep.simxGetObjectHandle(clientID,'Quadricopter_base',vrep.simx_opmode_blocking);
    
    [res,pose0(1:3,1)] = vrep.simxGetObjectPosition(clientID,Quadricopter_base,-1,vrep.simx_opmode_blocking);
    [res,pose0(4:6,1)] = vrep.simxGetObjectOrientation(clientID,Quadricopter_base,-1,vrep.simx_opmode_blocking);
    [res,velocity0(1:3,1),velocity0(4:6,1)] = vrep.simxGetObjectVelocity(clientID,Quadricopter_base,vrep.simx_opmode_blocking);
    velocity1 = zeros(6,1);
    t0 = 0;
    % init state
%     x0 = [0;0.25;4;0;0;0;0;0;0;0;0;0];
    x0 = [pose0;velocity0];
    % record states
%     xx(:,1) = x0;
    u0 = zeros(N,length_control);
    X0 = repmat(x0,1,N+1)';
    sim_tim = 200; % max simulation time
    % start NMPC
    nmpciter = 0;
    xx1 = [];
    u_c1= [];
    
    
%     %%Ô¤²âÄ¿±ê×´Ì¬
%     A = [0 0 1 0;0 0 0 1;0 0 0 0;0 0 0 0];  B = [0 0;0 0;1 0;0 1]; 
%     inter = dt;
%     lena = size(A);
%     lenb = size(B);
%     Ak = A*inter + eye(lena(1));
%     Bk = B*inter;
%     F = cell(N,1);
%     PHI = cell(N,N);
%     for i = 1:1:N        % ¼ÆËãÔ¤²â·½³Ì¾ØÕó
%       F{i,1} = Ak^i;
%     end
%     F = cell2mat(F);
% 
%     for i = 1:1:N
%        for j = 1:1:N
%            if (j<=i)
%                PHI{i,j} = Ak^(i-j)*Bk;
%            else
%                PHI{i,j} = zeros(lena(1),lenb(2));
%            end
%        end
%     end
%     PHI = cell2mat(PHI);
    %%Ô¤²âÄ¿±ê×´Ì¬
    A = [0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0];  
    B = [0 0 0;0 0 0;0 0 0;1 0 0;0 1 0;0 0 1]; 
    inter = 0.1;
    lena = size(A);
    lenb = size(B);
    Ak = A*inter + eye(lena(1));
    Bk = B*inter;
    F = cell(N,1);
    PHI = cell(N,N);
    for i = 1:1:N        % ¼ÆËãÔ¤²â·½³Ì¾ØÕó
      F{i,1} = Ak^i;
    end
    F = cell2mat(F);

    for i = 1:1:N
       for j = 1:1:N
           if (j<=i)
               PHI{i,j} = Ak^(i-j)*Bk;
           else
               PHI{i,j} = zeros(lena(1),lenb(2));
           end
       end
    end
    PHI = cell2mat(PHI);
    record_pose0 = [];
    record_pose1 = [];
    main_loop  = tic;
    w = [];
    while(vrep.simxGetConnectionId(clientID)~=-1 && nmpciter<sim_tim/dt)
        current_time = nmpciter*dt;
        % init state
        [res,pose0(1:3,1)] = vrep.simxGetObjectPosition(clientID,Quadricopter_base,-1,vrep.simx_opmode_blocking);
        [res,pose0(4:6,1)] = vrep.simxGetObjectOrientation(clientID,Quadricopter_base,-1,vrep.simx_opmode_blocking);
        [res,velocity0(1:3,1),velocity0(4:6,1)] = vrep.simxGetObjectVelocity(clientID,Quadricopter_base,vrep.simx_opmode_blocking);
        [res,pose1(1:3,1)] = vrep.simxGetObjectPosition(clientID,Bill_base,-1,vrep.simx_opmode_blocking);
        [res,pose1(4:6,1)] = vrep.simxGetObjectOrientation(clientID,Bill_base,-1,vrep.simx_opmode_blocking);
        pose1(3,1) = pose1(3,1)+2;
%         former_velocity1 = velocity1;
        [res,velocity1(1:3,1),velocity1(4:6,1)] = vrep.simxGetObjectVelocity(clientID,Bill_base,vrep.simx_opmode_blocking);
%         target_state = [pose1(1:3);velocity1(1:3)];
%         target_acc = (velocity1(1:2)-former_velocity1(1:2))/dt;
        record_pose0 = [record_pose0,pose0];
        record_pose1 = [record_pose1,pose1];
        target_x = (1:N)*dt*velocity1(1)+pose1(1);
        target_y = (1:N)*dt*velocity1(2)+pose1(2);
        target_z = (1:N)*dt*velocity1(3)+pose1(3);
%         reference_Nstate = F*target_state+PHI*repmat([0;0;0],N,1);

        referencestate = [target_x;...
                          target_y;...
                          target_z;...
                          zeros(3,N);...
                          ones(1,N)*velocity1(1);...
                          ones(1,N)*velocity1(2);...
                          ones(1,N)*velocity1(3);...
                          zeros(3,N)];
%         delete w;
%         w = plot(referencestate(3,:));
%         title(num2str(pose0(3)));
        x0 = [pose0;velocity0];
%         X0 = repmat(x0,1,N+1)';
        args.p(1:length_state,1) = x0;
        % reference
%         t_predict = current_time:T:current_time+T*(N-1);
%         referencestate = QuadrotorReferenceTrajectory(t_predict);
        
        
%         referencecontrol = 4.9*ones(4,N);
%         reference = [referencestate;referencecontrol];
        reference = referencestate;
        args.p(length_state+1:length_state*(N+1),1) = reshape(reference,length_state*N,1);
        % init value of OPT variables
        args.x0 = [reshape(X0',length_state*(N+1),1);reshape(u0',length_control*N,1)];
%         args.x0 = zeros(length_state*(N+1)+length_control*N,1);
%         args.x0 = reshape(u0',length_control*N,1);
        sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
%         xx1(:,1:length_state,nmpciter+1) = reshape(full(sol.x(1:length_state*(N+1)))',length_state,N+1)';
        u = reshape(full(sol.x(length_state*(N+1)+1:end))',length_control,N)';   
%         u = reshape(full(sol.x(1:end))',length_control,N)';
        % apply control , integration and  shift the solutionn
        uOptimal = u(1,:);
%         uOptimal
%         for i=1:4
%           vrep.simxSetFloatSignal(id,['u',num2str(i)],uOptimal(1,i),vrep.simx_opmode_oneshot);vrchk(vrep, res, true);
%         end
%         [res,Kf] = vrep.simxGetFloatSignal(clientID,'Kf',vrep.simx_opmode_oneshot)
%         [res,Km] = vrep.simxGetFloatSignal(clientID,'Km',vrep.simx_opmode_oneshot);
%         if Kf==0
%             particleVelocity1 = 0;
%             particleVelocity2 = 0;
%             particleVelocity3 = 0;
%             particleVelocity4 = 0;
%         else
            particleVelocity1 = uOptimal(1);
            particleVelocity2 = uOptimal(2);
            particleVelocity3 = uOptimal(3);
            particleVelocity4 = uOptimal(4);
%         end
        vrep.simxSetFloatSignal(clientID,'particleVelocity1',particleVelocity1,vrep.simx_opmode_oneshot);
        vrep.simxSetFloatSignal(clientID,'particleVelocity2',particleVelocity4,vrep.simx_opmode_oneshot);
        vrep.simxSetFloatSignal(clientID,'particleVelocity3',particleVelocity3,vrep.simx_opmode_oneshot);
        vrep.simxSetFloatSignal(clientID,'particleVelocity4',particleVelocity2,vrep.simx_opmode_oneshot);
%         [t0, x0, u0] = shift(dt, t0, x0, u,f);
% %         xx(:,nmpciter+2) = x0;
        u0 = [u(2:size(u,1),:);u(size(u,1),:)];
        X0 = reshape(full(sol.x(1:length_state*(N+1)))',length_state,N+1)'; % get solution TRAJECTORY
% %         % Shift trajectory to initialize the next step
        X0 = [X0(2:end,:);X0(end,:)];
        nmpciter = nmpciter + 1;
        vrep.simxSynchronousTrigger(clientID);
    end
    main_loop_time = toc(main_loop);
    average_mpc_time = main_loop_time/(nmpciter+1)
  
else
    disp('Failed connecting to remote API server1');
end
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(clientID);
vrep.delete();