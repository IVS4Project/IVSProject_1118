function type_def(csvFileName)
    fileDir = fileparts(mfilename('fullpath'));
    if nargin < 1
        csvFileName = 'type_def.csv';
    end
    if nargin < 2
        matFileName = append(fileDir,'\','bus_init_structs.mat');
    end
    
    %% === Read CSV ===
    opts = detectImportOptions(csvFileName, 'Delimiter', ',', 'NumHeaderLines', 0);
    opts.VariableNamesLine = 1;
    varNames = opts.VariableNames;

    % We enforce the order: BusName,ElementName,class,DataType,Dim,Min,Max,InitValue,Description
    expectedOrder = {'BusName','ElementName','class','DataType','Dim','Min','Max','InitValue','Description'};
    presentVars = intersect(expectedOrder, varNames, 'stable');  
    opts.SelectedVariableNames = presentVars;
    opts = setvartype(opts, presentVars, 'char');
    data = readtable(csvFileName, opts);

    hasInitValue = ismember('InitValue', data.Properties.VariableNames);
    hasDim      = ismember('Dim', data.Properties.VariableNames);

    %% === Collect Bus info ===
    busMap     = containers.Map();
    busInitMap = containers.Map();

    for i = 1:height(data)
        if strcmp(data.class{i}, 'numeric')
            
            be = Simulink.BusElement;
            be.Name     = data.ElementName{i};
            be.DataType = data.DataType{i};
            busName     = data.BusName{i};
            
            %% --- Parse Dimensions (NEW position: right after DataType) ---
            if hasDim && ~isempty(data.Dim{i})
                dimStr = strtrim(data.Dim{i});
                
                if contains(dimStr,'[')
                    dimVal = str2num(dimStr); %#ok<ST2NM>
                elseif contains(dimStr,' ')
                    dimVal = str2num(dimStr); %#ok<ST2NM>
                else
                    dimVal = str2double(dimStr);
                end
                
                if isempty(dimVal), dimVal = 1; end
                be.Dimensions = dimVal;
            else
                be.Dimensions = 1;
            end

            %% --- Min/Max/Description ---
            if ~isempty(data.Min{i}), be.Min = str2double(data.Min{i}); end
            if ~isempty(data.Max{i}), be.Max = str2double(data.Max{i}); end
            if ~isempty(data.Description{i}), be.Description = data.Description{i}; end

            %% --- InitValue ---
            if isKey(busInitMap, busName)
                initStruct = busInitMap(busName);
            else
                initStruct = struct();
            end

            if hasInitValue && ~isempty(data.InitValue{i})
                try
                    baseVal = cast(str2double(data.InitValue{i}), data.DataType{i});
                catch
                    baseVal = feval(data.DataType{i});
                end
            else
                baseVal = feval(data.DataType{i});
            end

            % dimension-aware default init
            if numel(be.Dimensions) == 1
                initValue = repmat(baseVal, be.Dimensions, 1);
            else
                initValue = repmat(baseVal, be.Dimensions);
            end

            initStruct.(be.Name) = initValue;
            busInitMap(busName) = initStruct;

            %% --- Append element to bus ---
            if isKey(busMap, busName)
                busMap(busName) = [busMap(busName), be];
            else
                busMap(busName) = be;
            end
        end
    end
    
    %% === Create Bus objects ===
    busNames = keys(busMap);
    for i = 1:length(busNames)
        busObj = Simulink.Bus;
        busObj.Elements = busMap(busNames{i});
        assignin('base', busNames{i}, busObj);
    end

    %% === Create init struct for each struct declaration ===
    initStructs = struct();

    for i = 1:height(data)
        if strcmp(data.class{i}, 'struct')
            originalName = data.ElementName{i};
            busType      = data.DataType{i};   % e.g. busMissionInfo
            varName      = ['init_' originalName];

            % bus 존재 여부 확인
            if evalin('base', sprintf('exist(''%s'',''var'')', busType)) ~= 1
                warning(['Bus type not found: ', busType]);
                continue;
            end

            % 기본 struct 생성
            try
                defaultStruct = Simulink.Bus.createMATLABStruct(busType);
            catch ME
                warning(['Failed to create struct for ', busType, ' — ', ME.message]);
                continue;
            end

            % 사용자 정의 InitValue override
            if isKey(busInitMap, busType)
                userInit = busInitMap(busType);
                flds = fieldnames(userInit);
                for f = 1:numel(flds)
                    defaultStruct.(flds{f}) = userInit.(flds{f});
                end
            end

            initStructs.(varName) = defaultStruct;
            assignin('base', varName, defaultStruct);
        end
    end

    %% === Save .mat ===
    save(matFileName, '-struct', 'initStructs');
    disp(['Saved init_ variables to: ', matFileName]);
end
