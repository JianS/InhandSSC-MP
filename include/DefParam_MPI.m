function DefParam_MPI
    clear global param
    global param
    
    param.t_int = 0.002;
    param.T = 20; % total time
    
    param.T1 = 5; % sticking phase time
    param.T2 = param.T - param.T1; % sliding phase time
    
    param.N = round(param.T/param.t_int);
    param.N1 = round(param.T1/param.t_int);
    param.N2 = param.N - param.N1;
    
    param.mu = 0.2502; 0.3743; % mu between the finger and the object 
    param.mu_e = 2; % mu between the object and the environment
    
    param.G = [0;-20]; % gravity force of the object
    param.p_g_B = [0; 0.1]; % center of mass position in B
    
    param.n_f = 2; % # of fingers
    
    param.n_e = 2; % # of external contact points
    param.p_e_B = zeros(2,param.n_e); % external contact pts pos in body frame
    param.nhat_e_B = zeros(2,param.n_e); % external contact normal in body frame
    
    
    % define anchor positions - w.r.t. Hand frame
    param.AnchorPos_H(:,1) = [-0.07; 0];
    param.AnchorPos_H(:,2) = [0.07; 0];
    
    % define stiffness of each finger
%     k = -100;
%     param.K_H(:,:,1) = [150 0; 0 100];
%     param.K_H(:,:,2) = [150 0; 0 100];
%     param.K_H(:,:,1) = [164.857 0; 0 113.657];
%     param.K_H(:,:,2) = [150.348 0; 0 99.995];
    param.K_H(:,:,1) = [152.0578 0; 0 101.0974];
    param.K_H(:,:,2) = [150.2263 0; 0 105.9395];
    
    % define equilibrium length of each finger
    param.d0(:,1) = [0.07 -0.05];
    param.d0(:,2) = [-0.07 -0.05];
end