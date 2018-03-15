function [ret,o] = CheckInit(o)
    Ro = CalR2d(o.q_o(3,1));
%             dRo = CaldR2d(o.q_o(3,1), o.dq_o(3,1));

    Rh = CalR2d(o.q_h(3,1));
%             dRh = CaldR2d(o.q_h(3,1), o.dq_h(3,1));
    i = 1;
    ret = 1;

    for j = 1:o.n_f
        [o.p_f_B(:,1,j), N_B, ~] = GetPtOnObj(o.p_f_B(2,1,j), j, o.obj_data);

        o.p_f(:,i,j) = o.q_o(1:2,i) + Ro*o.p_f_B(1:2,i,j);
        o.p_f_H(:,i,j) = Rh'*(o.q_o(1:2,i)-o.q_h(1:2,i)) + Rh'*Ro*o.p_f_B(:,i,j);

        d_H = o.p_f_H(:,i,j) - o.p_a_H(:,j) - o.param.d0(:,j);
%                 d = o.q_h(1:2,i) + Rh*d_H;

        o.f_c_H(:,i,j) = -o.param.K_H(:,:,j)*d_H;
        o.f_c(:,i,j) = Rh*o.f_c_H(:,i,j);

        Nhat_B = N_B/norm(N_B);
        Nhat = Ro*Nhat_B; %N = Ro*N_B;
        Nhat_H = Rh'*Nhat;

        if o.f_c(:,i,j)'*Nhat > 0
            disp(['i=' num2str(i) ', finger ' num2str(j) ' breaking contact!']);
        end
        o.f_N_H(:,i,j) = (o.f_c_H(:,i,j)'*Nhat_H)*Nhat_H;
        o.f_t_H(:,i,j) = o.f_c_H(:,i,j) - o.f_N_H(:,i,j);
        o.f_N(:,i,j) = Rh*o.f_N_H(:,i,j);
        o.f_t(:,i,j) = Rh*o.f_t_H(:,i,j);
        
        o.p_a(:,i,j) = o.q_h(1:2,i) + Rh*o.p_a_H(:,j);

        if norm(o.f_t(:,i,j)) >= o.param.mu*norm(o.f_N(:,i,j))
            disp(['finger ' num2str(j) ' initially not sticking!']);
            ret = 0;
        end

    end
    
    if ~(CheckForceBalance(o, 1)==1);
        disp('Initial force cannot balanced by the external contacts!');
        ret = 0;
    end
end