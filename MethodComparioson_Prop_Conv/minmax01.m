function y = minmax01(x)
    xmin = min(x(:));
    xmax = max(x(:));
    denom = xmax - xmin;

    if denom == 0
        y = zeros(size(x));   % 全候補が同じ値なら0
    else
        y = (x - xmin) ./ denom;
    end
end