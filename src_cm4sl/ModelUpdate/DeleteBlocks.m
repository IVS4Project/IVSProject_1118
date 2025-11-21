function [] = DeleteBlocks(subs, opts)
% Delete blocks of substitute systems defined in subs

for (i=1:numel(subs))
    blck      = subs(i).blck;
    trgSys    = subs(i).trgSys;
    trgBlck   = strcat(trgSys,'/',blck);

    if (isfield(opts,'DeleteLines') && opts.DeleteLines == 1)
        h = get_param(trgBlck,'LineHandles');
        % clear all lines
        DeleteLines(h);
    end

    delete_block (trgBlck);

end
end
