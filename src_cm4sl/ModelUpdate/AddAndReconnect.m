function [] = AddAndReconnect(subs)
% Add systems defined in subs and reconnect ports

for i = 1:numel(subs)
    blck      = subs(i).blck;
    blck_src  = subs(i).blck_src;
    srcSys    = subs(i).srcSys;
    trgSys    = subs(i).trgSys;
    srcBlck   = strcat(srcSys, '/',blck_src);
    trgBlck   = strcat(trgSys,'/',blck);

    % Save current ports and connections
    h         = get_param(trgBlck,'LineHandles');
    lines_old = SaveConnections(h);

    % Clear all connection lines
    DeleteLines(h);

    % Add block
    new_block = add_block(srcBlck, [trgSys '/' blck_src]);
    newBlock_Ports = get_param(new_block, 'PortHandles');

    % Reconnect
    add_line(trgSys, lines_old.out.SrcPortHandle, newBlock_Ports.Inport);
    add_line(trgSys, newBlock_Ports.Outport, lines_old.out.DstPortHandle);

    % Rearrange system blocks
    if (i == numel(subs))
	Simulink.BlockDiagram.arrangeSystem(trgSys)
    end
end
end
