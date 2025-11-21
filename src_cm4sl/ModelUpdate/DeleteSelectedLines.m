% Delete all lines attached to Inports and Outports
function [] = DeleteSelectedLines(h, OutportNums, InportNums)
% clear selected lines of the outport numbers stored in OutportNums
% and of the inport numbers stored in InportNums
    for (iP=1:numel(OutportNums))
        iH = OutportNums(iP);
        delete_line(h.Outport(iH));
    end
    for (iP=1:numel(InportNums))
        iH = InportNums(iP);
        delete_line(h.Inport(iH));
    end
end
