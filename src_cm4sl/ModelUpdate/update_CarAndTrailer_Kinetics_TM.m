subs = []; % used to store componets that will be replaced/reconnected
opts = [];
sys          = 'Kinetics';
ModifiedPort = -1;
PortKind     = 'Ignored';
ModSys    = 'Ignored';

Parent_src = 'generic_truck_src/TruckMaker/IPG Vehicle/CarAndTrailer';
Parent_trg = append(trg_mdl,'/TruckMaker/IPG Vehicle/CarAndTrailer');

Parent_trg_temp = append(trg_mdl,'/TruckMaker');
System_matches = FindSystem ('Vehicle');
% find system with given parent in trg system
for (j=1:numel(System_matches.trgBlck))
    if (strcmp(get_param(System_matches.trgBlck{j},'Name'),'Vehicle') && strcmp(get_param(System_matches.trgBlck{j}, 'Parent'), Parent_trg_temp))
        Parent_trg = append(trg_mdl,'/TruckMaker/Vehicle/IPG Vehicle/CarAndTrailer');
    end
end

% find all instances of the following Blocks
blck   = 'FSpring_ext';
subs_add = FindSubsFromParentSys_wPath(blck, sys, ModSys, ModifiedPort, PortKind, Parent_src, Parent_trg);
subs     = [subs, subs_add];

blck   = 'FDamp_ext';
subs_add = FindSubsFromParentSys_wPath(blck, sys, ModSys, ModifiedPort, PortKind, Parent_src, Parent_trg);
subs     = [subs, subs_add];

blck   = 'FBuf_ext';
subs_add = FindSubsFromParentSys_wPath(blck, sys, ModSys, ModifiedPort, PortKind, Parent_src, Parent_trg);
subs     = [subs, subs_add];

blck   = 'FStabi_ext';
subs_add = FindSubsFromParentSys_wPath(blck, sys, ModSys, ModifiedPort, PortKind, Parent_src, Parent_trg);
subs     = [subs, subs_add];

opts = [];
DeleteBlocks (subs, opts);


subs = [];
opts = [];
sfunparam    = 'Kinetics';
ModSys       = 'ignored';
ModifiedPort = -1; % replace the S-Function itself
PortKind     = 'Inport';

subs_add = FindSubsFromSFun (sfunparam, ModSys, ModifiedPort, PortKind);
subs     = [subs, subs_add];


opts.OldPortNumsIn  = [1:12];
opts.OldPortNumsOut = [1:6];
% opts.FontSizeLabel  = 2;
opts.AddTerms       = 1;

ReplaceAndReconnect(subs, opts);

clear opts subs;
