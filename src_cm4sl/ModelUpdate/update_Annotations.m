% Updating the annotation text containing CarMaker version (valid for TM4SL models as well)
annotationObjs = find_system(trg_mdl,'FindAll','on','Type','annotation');
for j=1:numel(annotationObjs)
    if regexp(get_param(annotationObjs(j),'PlainText'),OldVersion)
        annObj = get_param(annotationObjs(j),'Object');
        annObj.Text = NewVersion;
    end
end
