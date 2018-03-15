function [c,ceq] = constraints(mu)

%% define inequal constraints c(i)<=0
c(1) = -mu;

ceq=[];

end