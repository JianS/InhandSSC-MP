function [pos, N, dnhat_over_dpfB] = GetPtOnObj(h, LorR, obj_data)
% get the position of a point on the circle in Body frame
% 'h' -- the height (y) of the contact point in Body frame.
% 'LorR' -- tell which finger:
%        1 for the left finger;
%        2 for the right finger.


% n = size(obj_pts, 2);
% 
% [obj_i, iUP, iDOWN]= LookupObjPt(h, obj_pts);
% 
% x1 = obj_pts(1, iUP); y1 = obj_pts(2, iUP);
% x2 = obj_pts(1, iDOWN); y2 = obj_pts(2, iDOWN);

if LorR == 1
    a = obj_data.al;
elseif LorR == 2
    a = obj_data.ar;
else 
    disp('LorR can only be 1 or 2!!');
    LorR
    return
end

% x = (x2-x1)/(y2-y1)*(h-y1) + x1;
% n_a = length(a);
% x = 0;
% k = 0;% k=dx/dy
% for i = 1:n_a
%     x = x + a(i)*h^(i-1);
%     k = k+(i-1)*a(i)*h^(i-2);
%     if i==1 && h==0
%         k = 0;
%     end
% end
% pos = [x; h];

pos = CalPtPosOnObj(a, h);
% if(sum(pos==pos2)~=2)
%     disp('ddd');
% end

% calculate normal direction
% tt = [k;1]/norm([k; 1]);
% if LorR == 1
%     N = [-tt(2); tt(1)];
% elseif LorR == 2
%     N = [tt(2); -tt(1)];
% end

N = CalNormal(a, h, LorR);
% if(sum(N==N2)~=2)
%     disp('ddd');
% end

h_next = h - 2e-5;
pos_next = CalPtPosOnObj(a, h_next);
Nnext = CalNormal(a, h_next, LorR);

DNhat = Nnext - N;
Dx = pos_next(1)-pos(1);
Dy = h_next - h;


dnhat_over_dpfB = [DNhat(1)/Dx DNhat(1)/Dy;
                   DNhat(2)/Dx DNhat(2)/Dy];
% if LorR==2
%     dnhat_over_dpfB = [DNhat(1)/Dx DNhat(1)/Dy;
%                        DNhat(2)/Dx DNhat(2)/Dy];
% else
%     dnhat_over_dpfB = [-DNhat(1)/Dx DNhat(1)/Dy;
%                        -DNhat(2)/Dx DNhat(2)/Dy];
% end

% dnhat_over_dpfB = [1 1;1 1];

end

