function pos = CalPtPosOnObj(a, h)

    n_a = length(a);
    x = 0;
    for i = 1:n_a
        x = x + a(i)*h^(i-1);
    end
    pos = [x; h];

end