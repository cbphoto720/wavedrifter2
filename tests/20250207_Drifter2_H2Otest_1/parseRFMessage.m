function [success, drifter] = parseRFMessage(rfmessage)
    % Initialize drifter struct
    drifter = struct('Drifter_Status', '', 'lat', 0, 'lon', 0, 'lastUTC', 0, 'voltage', 0, 'RSSI', 0);

    % Define the expected pattern in the message
    pattern = '\[(\w+),(-?\d+),(-?\d+),(\d+),([\d\.]+)V\].*RSSI\s(-?\d+)';

    % Try to extract the values using regex
    tokens = regexp(rfmessage, pattern, 'tokens');

    if isempty(tokens)
        fprintf('Parsing failed.\n');
        success = false;
        return;
    end

    % Convert extracted values
    tokens = tokens{1};
    drifter.Drifter_Status = tokens{1};
    drifter.lat = int32(str2double(tokens{2}) * 10); % Convert to 1E7
    drifter.lon = int32(str2double(tokens{3}) * 10); % Convert to 1E7
    drifter.lastUTC = uint32(str2double(tokens{4}));
    drifter.voltage = int16(str2double(tokens{5}) * 1000); % Convert voltage to mV
    drifter.RSSI = int16(str2double(tokens{6})); % Convert RSSI to int16

    success = true;
end
