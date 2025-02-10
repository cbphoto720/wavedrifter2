function parsedStruct = parseCleanedFile(filename)
    % Open the file for reading
    fid = fopen(filename, 'r');
    if fid == -1
        error('Could not open file: %s', filename);
    end

    % Initialize an empty struct with correct fields
    parsedStruct = struct('Drifter_Status', {}, 'lat', {}, 'lon', {}, 'lastUTC', {}, 'voltage', {}, 'RSSI', {});

    % Read and parse each line
    index = 1;
    while ~feof(fid)
        line = fgetl(fid);
        if ischar(line)
            [success, drifter] = parseRFMessage(line);
            if success
                % Ensure the struct fields match before assignment
                parsedStruct(index) = orderfields(drifter);
                index = index + 1;
            end
        end
    end

    % Close the file
    fclose(fid);

    % Assign the struct dynamically in the workspace
%     assignin('base', structName, parsedStruct);
%     disp(['Struct "', structName, '" created successfully.']);
% end
