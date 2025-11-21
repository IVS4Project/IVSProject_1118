function [ValidForUpdate] = IsValidForUpdate(Mdl_FileName, ValidProd, OldVersion, NewVersion)
%Check if model can be updated
% - CM/TM previous model? ValidProd =  {'TruckMaker', 'CarMaker', 'MotorcycleMaker'}
% - not a backup file
% - not the reference file
% - not updated yet

SetGlobal;
ValidForUpdate = 0;
[Mdl_FilePath, Mdl_BaseName, Mdl_FileExt] = fileparts(Mdl_FileName);


% correct IPG Product
for i=1:numel(ValidProd)
    if ((numel(find_system(bdroot,'Name',ValidProd{i}))) ==1 )
        ValidForUpdate = 1;
    end
end
if (ValidForUpdate == 0)
    fprintf('Skipped: %s - no valid CM4SL or TM4SL model\n', Mdl_FileName);
    return;
end

% Reference = src file for model update
if (strcmp(Mdl_BaseName, ReferenceModel) == 1)
    fprintf('Skipped: %s - Reference file!\n', Mdl_FileName);
    ValidForUpdate = 0;
    return;
end

% backup or already updated
if (isempty(regexp(Mdl_FileName,['\w+\',postfix_backup,'\.\w+'])) ~= 1 || ...
    isempty(regexp(Mdl_FileName,['\w+\',postfix_updated,'\.\w+'])) ~= 1)
        fprintf('Skipped: %s - Backup file!\n', Mdl_FileName);
        ValidForUpdate = 0;
        return;
end

% previous CM/TM model
isOldCMVersion = 0;
annotationObjs = find_system(bdroot,'FindAll','on','Type','annotation');
for j=1:numel(annotationObjs)
    if regexp(get_param(annotationObjs(j),'PlainText'),NewVersion)
        fprintf('Skipped: %s - is a %s model. No update neccessary.\n', Mdl_FileName, NewVersion);
        ValidForUpdate = 0;
    end
    if regexp(get_param(annotationObjs(j),'PlainText'),OldVersion)
	isOldCMVersion = 1;
    end
end
if (~isOldCMVersion)
    fprintf('Skipped: %s - is not a %s model.\n', Mdl_FileName, OldVersion);
    ValidForUpdate = 0;
    return;
end

ValidForUpdate = 1;
end
