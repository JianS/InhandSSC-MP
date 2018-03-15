function [p_f, p_f_B, p_f_H, Ifslide, f_c, f_N, f_t, cost] = Simulate(h_init, q_o, dq_o,...
    q_h, dq_h, p_a, dp_a,  n, mu, param, obj_data)

%% Simulate the contact points trajectory

% set initial finger positions
% s = zeros(n, param.n_f);
% s(1,:) = s_init(:);

p_f = zeros(2, n, param.n_f);
p_f_B = zeros(2, n, param.n_f);
p_f_H = zeros(2, n, param.n_f);
thetaf = zeros(n, param.n_f);

p_f_B(2,1,:) = h_init;

dp_f = zeros(2, n, param.n_f);
dp_f_B = zeros(2, n, param.n_f);
dp_f_H = zeros(2, n, param.n_f);

f_c = zeros(2, n, param.n_f);
f_N = zeros(2, n, param.n_f);
f_t = zeros(2, n, param.n_f);
f_c_H = zeros(2, n, param.n_f);
f_N_H = zeros(2, n, param.n_f);
f_t_H = zeros(2, n, param.n_f);
% v1B = param.CornerPos_B(:,1);v2B = param.CornerPos_B(:,2);v3B = param.CornerPos_B(:,3);

Ifslide = zeros(n, param.n_f);

% dodLog = zeros(2,2,n,param.n_f);
% lambdaLog = zeros(n, param.n_f);

cost = 0;

for i = 1:n
    
    thetao = q_o(3,i);
    Ro = CalR2d(thetao);
    dRo = [-sin(thetao) -cos(thetao); cos(thetao) -sin(thetao)]*dq_o(3,i);
    
    thetah = q_h(3,i);
    Rh = CalR2d(thetah);
    dRh = [-sin(thetah) -cos(thetah); cos(thetah) -sin(thetah)]*dq_h(3,i);
    
    for j = 1:param.n_f
%         r0 = param.CornerPos_B(:, param.fedge(1,j));
%         r1 = param.CornerPos_B(:, param.fedge(2,j));
%         v_B = r1 - r0;
%         
%         if i==1
%             p_f_B(:,i,j) = GetPtPosOnCircle(s(i,j), j, param);
%             p_f(:,1,j) = q_o(1:2,1) + Ro*p_f_B(1:2,1,j);
%             p_f_H(:,i,j) = Rh'*(q_o(1:2,i)-q_h(1:2,i)) + Rh'*Ro*p_f_B(:,i,j);
% %             p_f_H(:,i,j) = Rh'*(p_f(:,1,j) - q_h(1:2,i));
%         end
        
        [p_f_B(:,i,j), N_B, dnhat_over_dpfB] = GetPtOnObj(p_f_B(2,i,j), j, obj_data);
%         dodLog(:,:,i,j) = dnhat_over_dpfB;
        
        p_f(:,i,j) = q_o(1:2,i) + Ro*p_f_B(1:2,i,j);
        p_f_H(:,i,j) = Rh'*(q_o(1:2,i)-q_h(1:2,i)) + Rh'*Ro*p_f_B(:,i,j);
%         p_f_H(:,i,j) = Rh'*(p_f(:,1,j) - q_h(1:2,i));
        
        p_a_H = Rh'*(p_a(:,i,j) - q_h(1:2,i));
        
        d_H = p_f_H(:,i,j) - p_a_H - param.d0(:,j);
        d = q_h(1:2,i) + Rh*d_H;
        
        f_c_H(:,i,j) = -param.K_H(:,:,j)*d_H;
        f_c(:,i,j) = Rh*f_c_H(:,i,j);
        
%         xf(:,j) = [cos(p_f(3,i,j)); sin(p_f(3,i,j))];
%         yf(:,j) = [-sin(p_f(3,i,j)); cos(p_f(3,i,j))];
              
%         if j == 1
%             N_B = p_f_B(:,i,j) - param.o1;
%         elseif j==2
%             N_B = p_f_B(:,i,j) - param.o2;
%         end
        Nhat_B = N_B/norm(N_B);
        Nhat = Ro*Nhat_B; %N = Ro*N_B;
        Nhat_H = Rh'*Nhat;
        
        thetaf(i,j) = atan(-Nhat(1)/Nhat(2));
        
%         if cross([f_c(:,i,j);0], [N;0])'*[0;0;1] < 0
        if f_c(:,i,j)'*Nhat > 0
            disp(['i=' num2str(i) ', finger ' num2str(j) ' breaking contact!']);
            cost = cost + 10;
        end
        f_N_H(:,i,j) = (f_c_H(:,i,j)'*Nhat_H)*Nhat_H;
        f_t_H(:,i,j) = f_c_H(:,i,j) - f_N_H(:,i,j);
        f_N(:,i,j) = Rh*f_N_H(:,i,j);
        f_t(:,i,j) = Rh*f_t_H(:,i,j);
        
%         dN = norm(f_t(:,i,j));
        
        c_f = dRh'*(q_o(1:2,i)-q_h(1:2,i)) + Rh'*(dq_o(1:2,i)-dq_h(1:2,i)) +...
            (dRh'*Ro+Rh'*dRo)*p_f_B(:,i,j);
        
        
        if norm(f_t(:,i,j)) < mu*norm(f_N(:,i,j)) % sticking
            dp_f_B(:,i,j) = 0;
            dp_f_H(:,i,j) = c_f;
            dp_f(:,i,j) = dq_o(1:2,i) + dRo*p_f_B(1:2,i,j);
            
            Ifslide(i,j) = 0;
        else % sliding (possible)
            if i==1
%                 error(['finger ' num2str(j) ' initially not sticking']);
                 disp(['finger ' num2str(j) ' initially not sticking']);
                 cost = cost +10;
            end
            
%             e = dq_o(1:2,i) + dRo*p_f_B(1:2,i,j);
            lambda = Callambda(c_f, dnhat_over_dpfB, -Nhat, Ro, Rh, dRh, d, f_N(:,i,j), f_t(:,i,j), f_c(:,i,j), dp_a(:,i,j),mu, param, j);
%             Callambda(cf, N, Nhat, Ro, fN, ft, fc, dp_a, mu, param, j)
%             lambdaLog(i,j) = lambda;

            dp_f_B(:,i,j) = lambda*Ro'*Rh*f_t_H(:,i,j);
            dp_f_H(:,i,j) = c_f + lambda*f_t_H(:,i,j);
            dp_f(:,i,j) = dq_h(1:2,i) + dRh*p_f_B(:,i,j) + Rh*dp_f_H(:,i,j);
            
%             if (dp_f(:,i,j)'*[1;1]~=0)||(dq_o(:,i)'*[1;1;1]~=0)
            if norm(dp_f_B(:,i,j)) > 0 % sliding when relative motion is not zero
                Ifslide(i,j) = 1;
            else
                disp(['At friction cone but not sliding, lambda:' num2str(lambda) ', i:' num2str(i)]);
            end
        end
        
        
        if i< n
%             p_f(:,i+1,j) = p_f(1:2,i,j) + dp_f(:,i,j)*param.t_int;

            p_f_B(:,i+1,j) = p_f_B(:,i,j) + dp_f_B(:,i,j)*param.t_int;
%             p_f_H(:,i+1,j) = p_f_H(1:2,i,j) + dp_f_H(:,i,j)*param.t_int;
            
%             Dangle = norm(dp_f_B(:,i,j))*param.t_int/param.r;
%             testv = cross([Nhat_B' 0], [dp_f_B(:,i,j)' 0]);
%             if testv(3) < 0
%                 Dangle = -Dangle;
%             end
%             s(i+1,j) = s(i,j) + Dangle/param.theta_Per_s(j);
        end
    end
    
%     disp('diff of dod: ')
%     disp(num2str(dodLog(:,:,i,1)-dodLog(:,:,i,2)))

end

%%

% SimplePlot3(t_cam, s', 'Simulated finger contact relative location on edges "s_i" ')

end
