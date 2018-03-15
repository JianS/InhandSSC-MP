function DrawSpring2D( A, B, w, ratio, n, color, linewidth )
% draw springs with details
% A, B -  pos of two ends, coloum vectors!
% w - thickness of the spring
% ratio - how long the curved section is of the whole spring
% n - how many curals of the curved section
if ratio <= 0 || ratio > 1
    error(['Ratio of the curved spring invalid!  ration = ' num2str(ratio)]);
end

   pts = A; % points for plot
   l = norm(A-B);
   
   ab = (B - A)/l;
   abP = [-ab(2); ab(1)]; % perpenticular to ab
   AA = A + (1-ratio)/2*l*ab;
   pts = [pts AA];
   
   ls = l*ratio/(4*n);
%    last_pt = A';
   for i = 1:n
        pts = [pts pts(:,end)+ab*ls+w/2*abP];
        pts = [pts pts(:,end)+ab*2*ls-w*abP];
        pts = [pts pts(:,end)+ab*ls+w/2*abP];
   end
   
    pts = [pts B];
    
    line(pts(1,:),pts(2,:),'color',color,'linewidth',linewidth);

end

