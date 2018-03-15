function DrawSquare(x,y,r,linecolor, linestyle, fillcolor)

%     ang=0:0.1:2*pi; 
    xp=[-r r r -r -r];
    yp=[r r -r -r r];
%     plot(x+xp,y+yp, linecolor, 'linestyle',linestyle);
    fill(x+xp,y+yp, fillcolor, 'edgecolor', linecolor, 'linestyle',linestyle)
end
