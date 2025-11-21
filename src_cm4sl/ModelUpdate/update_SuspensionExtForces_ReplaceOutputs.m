function [] = update_SuspensionExtForces_ReplaceOutputs(src_mdl, trg_mdl)
subs = []; % used to store componets that will be replaced/reconnected
opts = [];
sys          = 'External Suspension Forces';
ModifiedPort = -1;
PortKind     = 'Ignored';
ModSys    = 'Ignored';

Parent_src = 'generic_src/CarMaker/IPG Vehicle/CarAndTrailer/Forces/External Suspension Forces';
Parent_trg = append(trg_mdl,'/CarMaker/IPG Vehicle/CarAndTrailer/Forces');

Parent_trg_temp = append(trg_mdl,'/CarMaker');
System_matches = FindSystem ('Vehicle');
% find system with given parent in trg system
for (j=1:numel(System_matches.trgBlck))
    if (strcmp(get_param(System_matches.trgBlck{j},'Name'),'Vehicle') && strcmp(get_param(System_matches.trgBlck{j}, 'Parent'), Parent_trg_temp))
        Parent_trg = append(trg_mdl,'/CarMaker/Vehicle/IPG Vehicle/CarAndTrailer/Forces');
    end
end

% find all instances of the following Blocks
blck   = 'FSpring_ext';

subs_add = FindSubsFromParentSys_wPath(blck, sys, ModSys, ModifiedPort, PortKind, Parent_src, Parent_trg);
subs     = [subs, subs_add];

opts.OldPortNumsIn  = [1];
opts.OldPortNumsOut = [1];
% opts.FontSizeLabel  = 2;
opts.AddTerms       = 1;

ReplaceAndReconnect(subs, opts);

clear opts subs;

subs = []; % used to store componets that will be replaced/reconnected
opts = [];

% find all instances of the following Blocks
blck   = 'FDamp_ext';

subs_add = FindSubsFromParentSys_wPath(blck, sys, ModSys, ModifiedPort, PortKind, Parent_src, Parent_trg);
subs     = [subs, subs_add];

opts.OldPortNumsIn  = [1];
opts.OldPortNumsOut = [1];
% opts.FontSizeLabel  = 2;
opts.AddTerms       = 1;

ReplaceAndReconnect(subs, opts);

clear opts subs;

subs = []; % used to store componets that will be replaced/reconnected
opts = [];

% find all instances of the following Blocks
blck   = 'FStabi_ext';

subs_add = FindSubsFromParentSys_wPath(blck, sys, ModSys, ModifiedPort, PortKind, Parent_src, Parent_trg);
subs     = [subs, subs_add];

opts.OldPortNumsIn  = [1];
opts.OldPortNumsOut = [1];
% opts.FontSizeLabel  = 2;
opts.AddTerms       = 1;

ReplaceAndReconnect(subs, opts);

clear opts subs;

subs = []; % used to store componets that will be replaced/reconnected
opts = [];


% find all instances of the following Blocks
blck   = 'FBuf_ext';

subs_add = FindSubsFromParentSys_wPath(blck, sys, ModSys, ModifiedPort, PortKind, Parent_src, Parent_trg);
subs     = [subs, subs_add];

opts.OldPortNumsIn  = [1];
opts.OldPortNumsOut = [1];
% opts.FontSizeLabel  = 2;
opts.AddTerms       = 1;

ReplaceAndReconnect(subs, opts);

clear opts subs;

sys = 'Forces';
blck          = 'External Suspension Forces';
ModifiedPort = -1;
PortKind     = 'Ignored';
ModSys    = 'Ignored';

Parent_src = 'generic_src/CarMaker/IPG Vehicle/CarAndTrailer/Forces';
Parent_trg = append(trg_mdl,'/CarMaker/IPG Vehicle/CarAndTrailer');

Parent_trg_temp = append(trg_mdl,'/CarMaker');
System_matches = FindSystem ('Vehicle');
% find system with given parent in trg system
for (j=1:numel(System_matches.trgBlck))
    if (strcmp(get_param(System_matches.trgBlck{j},'Name'),'Vehicle') && strcmp(get_param(System_matches.trgBlck{j}, 'Parent'), Parent_trg_temp))
        Parent_trg = append(trg_mdl,'/CarMaker/Vehicle/IPG Vehicle/CarAndTrailer');
    end
end

subsys = FindSubsFromParentSys_wPath(blck, sys, ModSys, ModifiedPort, PortKind, Parent_src, Parent_trg);
subsysName = append(subsys.trgSys,"/External Suspension Forces");
set_param(subsysName, 'Name', "External Suspension Forces LEGACY");
