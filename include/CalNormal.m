function  N = CalNormal( a, h, LorR )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    n_a = length(a);
%     x = 0;
    k = 0;% k=dx/dy
    for i = 1:n_a
%         x = x + a(i)*h^(i-1);
        k = k+(i-1)*a(i)*h^(i-2);
        if i==1 && h==0
            k = 0;
        end
    end
%     pos = [x; h];

    % calculate normal direction
    tt = [k;1]/norm([k; 1]);
    if LorR == 1
        N = [-tt(2); tt(1)];
    elseif LorR == 2
        N = [tt(2); -tt(1)];
    end

end

