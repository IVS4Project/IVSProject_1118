subs = [];
opts = [];


sfunparam    = 'cm_ptmotorcu_in';
ModSys       = 'CreateBus MCU_Cfg ISG';
ModifiedPort = 2; % Cfg ISG
PortKind     = 'Outport';

subs_add = FindSubsFromSFun (sfunparam, ModSys, ModifiedPort, PortKind);
subs     = [subs, subs_add];

sfunparam    = 'cm_ptmotorcu_in';
ModSys       = 'CreateBus MCU_Cfg Motor0';
ModifiedPort = 3; % Cfg Motor
PortKind     = 'Outport';

subs_add = FindSubsFromSFun (sfunparam, ModSys, ModifiedPort, PortKind);
subs     = [subs, subs_add];

sfunparam    = 'cm_ptmotorcu_in';
ModSys       = 'CreateBus MCU_Cfg Motor1';
ModifiedPort = 4; % Cfg Motor
PortKind     = 'Outport';

subs_add = FindSubsFromSFun (sfunparam, ModSys, ModifiedPort, PortKind);
subs     = [subs, subs_add];

sfunparam    = 'cm_ptmotorcu_in';
ModSys       = 'CreateBus MCU_Cfg Motor2';
ModifiedPort = 5; % Cfg Motor
PortKind     = 'Outport';

subs_add = FindSubsFromSFun (sfunparam, ModSys, ModifiedPort, PortKind);
subs     = [subs, subs_add];

sfunparam    = 'cm_ptmotorcu_in';
ModSys       = 'CreateBus MCU_Cfg Motor3';
ModifiedPort = 6; % Cfg Motor
PortKind     = 'Outport';

subs_add = FindSubsFromSFun (sfunparam, ModSys, ModifiedPort, PortKind);
subs     = [subs, subs_add];

opts.OldPortNumsIn  = [1];
opts.OldPortNumsOut = [1];
% opts.FontSizeLabel  = 2;
opts.AddTerms       = 1;

ReplaceAndReconnect(subs, opts);

clear opts subs;

