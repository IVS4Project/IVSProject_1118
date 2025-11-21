function y = lowpass_filter_mfile(u)
    persistent y_prev;
    alpha = 0.95;

    if isempty(y_prev)
        y_prev = u;
    end

    y = (alpha)*y_prev + (1-alpha)*u;
    y_prev = y;
end