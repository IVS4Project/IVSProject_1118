% Delete selected lines attached to Inports and Outports of blck in subsystem sub
function [] = DeleteSelectedLinesOfSub(sub, blck, OutportNums, InportNums)
% clear selected lines of the outport numbers stored in OutportNums
% and of the inport numbers stored in InportNums of the subsystem sub
if (numel(sub) ~= 0)
    trgSys    = sub.trgSys;
    trgBlck   = strcat(trgSys,'/',blck);
    h         = get_param(trgBlck,'LineHandles');

    % clear selected lines
    DeleteSelectedLines(h, OutportNums, InportNums);
end
end
