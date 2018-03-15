function circle(x,y,r,linecolor, linestyle, fillcolor, linewidth)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)

if nargin == 6
    linewidth = 1;
end

    ang=0:0.1:2*pi; 
    xp=r*cos(ang);
    yp=r*sin(ang);
%     plot(x+xp,y+yp, linecolor, 'linestyle',linestyle);
    fill(x+xp,y+yp, fillcolor, 'edgecolor', linecolor,...
        'linestyle',linestyle, 'linewidth', linewidth )
end
