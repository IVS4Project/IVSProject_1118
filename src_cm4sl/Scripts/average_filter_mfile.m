function [y, i_out] = average_filter_mfile(u)
    persistent buffer i cnt
    N = 50;

    if isempty(buffer)
        buffer = zeros(1, N);
        i = 0;
        cnt = 0;
    end

    i = i + 1;
    if i > N
        i = 1;
    end
    buffer(i) = u;

    if cnt < N
        cnt = cnt + 1;
    end

    y = mean(buffer(1:cnt));
    i_out = i;
end
