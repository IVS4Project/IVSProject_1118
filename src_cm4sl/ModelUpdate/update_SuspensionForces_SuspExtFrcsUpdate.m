subs = []; % used to store componets that will be replaced/reconnected
opts = [];

sfunparam    = 'ForcesSuspUpd';
ModSys       = 'Suspension External Forces Update';
ModifiedPort = 3;
PortKind     = 'Inport';


subs_add = FindSubsFromSFun (sfunparam, ModSys, ModifiedPort, PortKind);
subs     = [subs, subs_add];


opts.OldPortNumsIn  = [1];
opts.OldPortNumsOut = [1];
% opts.FontSizeLabel  = 2;
opts.AddTerms       = 1;
opts.PosFromSrc     = 1;
opts.NameFromSrc    = 1;

ReplaceAndReconnect(subs, opts);

clear opts subs;
clear subs;
