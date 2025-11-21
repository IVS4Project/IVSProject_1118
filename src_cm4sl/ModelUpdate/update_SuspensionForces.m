subs = [];
opts = [];


sfunparam    = 'ForcesSuspControl';
ModSys       = 'ignored';
ModifiedPort = -1; % replace the S-Function itself
PortKind     = 'ignored';

subs_add = FindSubsFromSFun (sfunparam, ModSys, ModifiedPort, PortKind);

subs     = [subs, subs_add];


opts.OldPortNumsIn  = [1];
opts.OldPortNumsOut = [1:2, -1, -1];
% opts.FontSizeLabel  = 2;
opts.AddTerms       = 1;

ReplaceAndReconnect(subs, opts);

clear opts subs;

subs = [];
opts = [];

sfunparam    = 'ForcesSusp';
ModSys       = 'ignored';
ModifiedPort = -1; % replace the S-Function itself
PortKind     = 'ignored';

subs_add = FindSubsFromSFun (sfunparam, ModSys, ModifiedPort, PortKind);
subs     = [subs, subs_add];

opts.OldPortNumsIn  = [1,3, -1, -1];
opts.OldPortNumsOut = [1:2, -1, -1];
% opts.FontSizeLabel  = 2;
opts.AddTerms       = 1;

ReplaceAndReconnect(subs, opts);

clear opts subs;

subs = [];
opts = [];

sfunparam    = 'ForcesSuspUpd';
ModSys       = 'ignored';
ModifiedPort = -1; % replace the S-Function itself
PortKind     = 'ignored';

subs_add = FindSubsFromSFun (sfunparam, ModSys, ModifiedPort, PortKind);
subs     = [subs, subs_add];

opts.OldPortNumsIn  = [1:2, -1, -1, -1];
opts.OldPortNumsOut = [1];
% opts.FontSizeLabel  = 2;
opts.AddTerms       = 1;

ReplaceAndReconnect(subs, opts);

clear opts subs;
