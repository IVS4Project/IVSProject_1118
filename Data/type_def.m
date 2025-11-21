function type_def(csvFileName)
    fileDir = fileparts(mfilename('fullpath'));
    if nargin < 1
        csvFileName = 'type_def.csv';
    end
    if nargin < 2
        matFileName = append(fileDir,'\','bus_init_structs.mat');
    end
    
    % === Read CSV ===
    opts = detectImportOptions(csvFileName, 'Delimiter', ',', 'NumHeaderLines', 0);
    opts.VariableNamesLine = 1;
    varNames = opts.VariableNames;

    if ismember('InitValue', varNames)
        varsToRead = {'BusName','ElementName','class','DataType','Min','Max','Description','InitValue'};
        hasInitValue = true;
    else
        varsToRead = {'BusName','ElementName','class','DataType','Min','Max','Description'};
        hasInitValue = false;
    end

    opts.SelectedVariableNames = varsToRead;
    opts = setvartype(opts, varsToRead, 'char');
    data = readtable(csvFileName, opts);

    % === Collect Bus info ===
    busMap = containers.Map();
    busInitMap = containers.Map();

    for i = 1:height(data)
        if strcmp(data.class{i}, 'numeric')
            busName = data.BusName{i};
            be = Simulink.BusElement;
            be.Name = data.ElementName{i};
            be.DataType = data.DataType{i};
            if ~isempty(data.Min{i}), be.Min = str2double(data.Min{i}); end
            if ~isempty(data.Max{i}), be.Max = str2double(data.Max{i}); end
            if ~isempty(data.Description{i}), be.Description = data.Description{i}; end

            if isKey(busInitMap, busName)
                initStruct = busInitMap(busName);
            else
                initStruct = struct();
            end

            % Safe InitValue parsing
            if hasInitValue && ~isempty(data.InitValue{i})
                try
                    initValue = cast(str2double(data.InitValue{i}), data.DataType{i});
                catch
                    initValue = feval(data.DataType{i});
                end
            else
                initValue = feval(data.DataType{i});
            end

            initStruct.(be.Name) = initValue;
            busInitMap(busName) = initStruct;

            if isKey(busMap, busName)
                busMap(busName) = [busMap(busName), be];
            else
                busMap(busName) = be;
            end
        end
    end

    % === Create Bus objects in base workspace ===
    busNames = keys(busMap);
    for i = 1:length(busNames)
        busName = busNames{i};
        busObj = Simulink.Bus;
        busObj.Elements = busMap(busName);
        assignin('base', busName, busObj);
    end

    % === Create init structures and assign to workspace with init_ prefix ===
    initStructs = struct();

    for i = 1:height(data)
        if strcmp(data.class{i}, 'struct')
            originalName = data.ElementName{i};     % e.g., ExampleBus_1
            busType = data.DataType{i};             % e.g., bus_ExampleBus
            varName = ['init_' originalName];       % e.g., init_ExampleBus_1

            if evalin('base', sprintf('exist(''%s'', ''var'')', busType)) ~= 1
                warning(['Bus type not found: ', busType]);
                continue;
            end

            try
                defaultStruct = Simulink.Bus.createMATLABStruct(busType);
            catch ME
                warning(['Failed to create struct for ', busType, ' — ', ME.message]);
                continue;
            end

            if isKey(busInitMap, busType)
                userInit = busInitMap(busType);
                fields = fieldnames(userInit);
                for f = 1:numel(fields)
                    if isfield(defaultStruct, fields{f})
                        defaultStruct.(fields{f}) = userInit.(fields{f});
                    end
                end
            end

            % Fill empty/invalid fields
            flds = fieldnames(defaultStruct);
            for k = 1:numel(flds)
                val = defaultStruct.(flds{k});
                if isempty(val) || ~isnumeric(val)
                    defaultStruct.(flds{k}) = 0;
                end
            end

            % Save and assign with init_ prefix
            initStructs.(varName) = defaultStruct;
            assignin('base', varName, defaultStruct);
        end
    end

    % === Save all init structures to .mat file ===
    save(matFileName, '-struct', 'initStructs');
    disp([' Saved init_ variables to .mat and assigned to base workspace: ', matFileName]);
end