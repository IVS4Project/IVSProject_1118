function [] = update_CarAndTrailer_Forces_CleanUp_TM(src_mdl, trg_mdl)


subs = []; % used to store componets that will be replaced/reconnected
opts = [];
sys          = 'CarAndTrailer';
ModifiedPort = -1; 
PortKind     = 'Ignored';
ModSys    = 'Ignored';

Parent_src = 'generic_src/TruckMaker/IPG Vehicle';
Parent_trg = append(trg_mdl,'/TruckMaker/IPG Vehicle');

Parent_trg_temp = append(trg_mdl,'/TruckMaker');
System_matches = FindSystem ('Vehicle');
% find system with given parent in trg system
for (j=1:numel(System_matches.trgBlck))
    if (strcmp(get_param(System_matches.trgBlck{j},'Name'),'Vehicle') && strcmp(get_param(System_matches.trgBlck{j}, 'Parent'), Parent_trg_temp))
        Parent_trg = append(trg_mdl,'/TruckMaker/Vehicle/IPG Vehicle');
    end
end

% find all instances of the following Blocks
blck   = 'Forces';
sub = FindSubsFromParentSys_wPath(blck, sys, ModSys, ModifiedPort, PortKind, Parent_src, Parent_trg);

% clear selected lines of subsystem sub
OutportNums = [5, 6, 7, 8];
InportNums = [];
DeleteSelectedLinesOfSub(sub, blck, OutportNums, InportNums);


subs = []; % used to store componets that will be replaced/reconnected
opts = [];

sys          = 'Forces';
ModifiedPort = -1; 
PortKind     = 'Ignored';
ModSys    = 'Ignored';


Parent_src = 'generic_src/TruckMaker/IPG Vehicle/CarAndTrailer';
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
    
opts.DeleteLines = 1;
DeleteBlocks (subs, opts);


