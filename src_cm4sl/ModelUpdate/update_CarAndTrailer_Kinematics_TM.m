subs = [];
opts = [];


sfunparam    = 'Kinematics';
ModSys       = 'ignored';
ModifiedPort = -1; % replace the S-Function itself
PortKind     = 'ignored';

subs_add = FindSubsFromSFun (sfunparam, ModSys, ModifiedPort, PortKind);
subs     = [subs, subs_add];


opts.OldPortNumsIn  = [1:2];
opts.OldPortNumsOut = [1:9, -1];
% opts.FontSizeLabel  = 2;
opts.AddTerms       = 1;

ReplaceAndReconnect(subs, opts);

clear opts subs;
