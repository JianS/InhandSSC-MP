function M = CalCubicPolyMatrix(T)
 M = [1 0 0 0;
      0 1 0 0;
      1 T T^2 T^3;
      0 1 2*T 3*T^2];

end