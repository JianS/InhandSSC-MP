function o = BuildSticking(o)
   n1 = o.param.N1;

   o.fedge = zeros(2, o.n_f);

   for j = 1:o.n_f
       dy = o.yfB_goal(j) - o.p_f_B(2,1,j);

       fN = o.f_N(:,1,j);
       tt = [-sign(fN(1))*fN(2); sign(fN(1))*fN(1)];

       o.fedge(:,j) = fN + sign(dy)*o.param.mu*tt;

   end

   A = zeros(2);
   B = zeros(2,1);
   % here assuming hand do not rotate, and Rh = I
   for j = 1:o.n_f
       tt = [o.fedge(2,j); -o.fedge(1,j)];
       A(j,:) = tt'*o.param.K_H(:,:,j);
       c = o.p_f(:,1,j) - o.p_a_H(:,j) - o.param.d0(:,j);
       B(j) = tt'*o.param.K_H(:,:,j)*c;
   end

   ph = A\B; % calculate the hand pos at the end of the sticking phase
%    vh = (ph - o.q_h(1:2,1) )/(o.param.N1-1); % constant speed
   
   M1 = o.CalCubicPolyMatrix(n1-1);% cubic traj with zero vel at start/end
   a1 = M1\[o.q_h(1,1); 0; ph(1); 0];
   a2 = M1\[o.q_h(2,1); 0; ph(2); 0];
   
   % start simulating 
   N_B = zeros(2, o.n_f);
   for j = 1:o.n_f
       [~, N_B(:,j), ~] = GetPtOnObj(o.p_f_B(2,1,j), j, o.obj_data);
       N_B(:,j) = N_B(:,j)/norm(N_B(:,j));
   end

   for i = 1:n1
%        if i > 1
%            o.q_h(1:2,i) = o.q_h(1:2,i-1) + vh;
%        end
       t = i-1;
       tvec = [1 t t^2 t^3];
       o.q_h(1:2,i) = [tvec*a1; tvec*a2];
       
%        o.dq_h(1:2,i) = vh/o.param.t_int;
       tvecV = [0 1 2*t 3*t^2];
       o.dq_h(1:2,i) = [tvecV*a1; tvecV*a2];
       
       Ro = CalR2d(o.q_o(3,i));
       Rh = CalR2d(o.q_h(3,i));
       for j = 1:o.n_f
           if i > 1
               o.p_f_B(:,i,j) = o.p_f_B(:,i-1,j);
               o.p_f(:,i,j) = o.p_f(:,i-1,j);
               o.p_f_H(:,i,j) = Rh'*(o.q_o(1:2,i)-o.q_h(1:2,i))...
                   + Rh'*Ro*o.p_f_B(:,i,j);

               d_H = o.p_f_H(:,i,j) - o.p_a_H(:,j) - o.param.d0(:,j);

               o.f_c_H(:,i,j) = -o.param.K_H(:,:,j)*d_H;
               o.f_c(:,i,j) = Rh*o.f_c_H(:,i,j);

%                        Nhat_B = N_B/norm(N_B);
               Nhat = Ro*N_B(:,j); %N = Ro*N_B;
               Nhat_H = Rh'*Nhat;

               if o.f_c(:,i,j)'*Nhat > 0
                    disp(['i=' num2str(i) ', finger ' num2str(j) ' breaking contact!']);
               end
               o.f_N_H(:,i,j) = (o.f_c_H(:,i,j)'*Nhat_H)*Nhat_H;
               o.f_t_H(:,i,j) = o.f_c_H(:,i,j) - o.f_N_H(:,i,j);
               o.f_N(:,i,j) = Rh*o.f_N_H(:,i,j);
               o.f_t(:,i,j) = Rh*o.f_t_H(:,i,j);

           end
           o.p_a(:,i,j) = o.q_h(1:2,i) + Rh*o.p_a_H(:,j);
       end
   end
   
   if ~(CheckForceBalance(o, n1)==1)
       error(['i=' num2str(i) ', forces cannot be balanced by the external contacts!']);
%        return
%        break;
   end

end