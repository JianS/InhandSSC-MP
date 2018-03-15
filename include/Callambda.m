function lambda = Callambda( cf, dnhat_over_dpfB, Nhat, Ro, Rh,dRh, d, fN, ft, fc, dp_a, mu, param, j)

%     mu = param.mu;
%     kx = param.K_H(1,1,j);
%     ky = param.K_H(2,2,j);
    
% (mu^2+1)*sin(2*thetah)*(sin(2*thetaf)*(fc(1)*(kx*(dp_a(1)-e(1))+2*fc(2)*(dthetah-dthetaf))+ky*fc(2)*(e(2)-dp_a(2)))+cos(2*thetaf)*(kx*fc(2)*(e(1)-dp_a(1))-dp_a(2)*ky*fc(1)+e(2)*ky*fc(1)-(fc(1)^2-fc(2)^2)*(dthetaf-dthetah)))+...
% (mu^2-1)*(kx*fc(1)*(e(1)-dp_a(1))+ky*fc(2)*(e(2)-dp_a(2)));
% 
% den = (fc(1)*cos(thetaf-thetah)+fc(2)*sin(thetaf-thetah))*(kx*cos(thetaf-thetah)*(fc(1)*((mu^2+1)*cos(2*(thetaf-thetah))-mu^2+1)+(mu^2+1)*fc(2)*sin(2*(thetaf-thetah)))+ky*sin(thetaf-thetah)*((mu^2+1)*fc(1)*sin(2*(thetaf-thetah))-fc(2)*((mu^2+1)*cos(2*(thetaf-thetah))+mu^2-1)));
% 
%     lambda = num/den;

% dd = norm(N);
% dnhat_over_dpfB = [1/dd-N(1)^2/(dd^3), -N(1)*N(2)/(dd^3);
%                    -N(1)*N(2)/(dd^3),  1/dd-N(2)^2/(dd^3)];
               
gn = dnhat_over_dpfB*Ro'*ft;

K_H = param.K_H(:,:,j);
K = Rh'*K_H*Rh;

gc = -K*ft;
% cc = K*dp_a - K*cf;
cc = K*dp_a - K*cf - (dRh'*K_H*Rh+Rh'*K_H*dRh)*d;

gN = gc'*Nhat*Nhat + fc'*gn*Nhat + fc'*Nhat*gn;
cN = cc'*Nhat*Nhat;

gt = gc - gN;
ct = cc - cN;

% this is the equation (18) in the paper that before simplification 
% lambda = (mu^2*fN'*cN - ft'*ct)/(ft'*gt - mu^2*fN'*gN);

den = ft'*gt - mu^2*fN'*gN;
a = mu^2*fN - ft;

g = a'*K/den;

h = K*cf + (dRh'*K_H*Rh+Rh'*K_H*dRh)*d;
c = a'*h/den;

% equation (19)
lambda = g*dp_a -c;


end
