subs = [];
opts = [];


sfunparam    = 'cm_ptcontrol_in';
ModSys       = 'CreateBus PTC_Cfg ISG1';
ModifiedPort = 5; % Cfg ISG
PortKind     = 'Outport';

subs_add = FindSubsFromSFun (sfunparam, ModSys, ModifiedPort, PortKind);
subs     = [subs, subs_add];

sfunparam    = 'cm_ptcontrol_in';
ModSys       = 'CreateBus PTC_Cfg Motor1';
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
