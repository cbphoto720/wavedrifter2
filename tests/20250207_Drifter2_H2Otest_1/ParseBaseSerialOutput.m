close all; clear all; clc
addpath(genpath('C:\Users\Cars\Documents\MATLAB\CPG\20250207_Drifter2_H2Otest_1'));

% run  this code once to generate purged data files from the base staiton
%% Grab files

% Define input and output file names
% inputfile=uigetfile('.log');
% inputfile=convertCharsToStrings(inputfile);
% inputFile = '51control';  % Replace with your actual input file
% inputFile = '54wet_float';  % Replace with your actual input file
% inputFile = '55wet_submerged-surface';  % Replace with your actual input file
% inputFile = '56wet_submerged-DEEP';  % Replace with your actual input file
% inputFile = 'LASTwet_submerged-DEEPBLUE';  % Replace with your actual input file

inputFile=strcat(inputFile,'.log');


outputFile = strcat(inputFile(1:end-4),'_cleaned.log'); % Output file

%% 

% Define the keyword and number of additional lines to copy
keyword = 'Drifter 1';
n = 0;  % Change this to the number of lines you want to copy after each match

% Open the input file for reading
fid_in = fopen(inputFile, 'r');
if fid_in == -1
    error('Could not open input file.');
end

% Open the output file for writing
fid_out = fopen(outputFile, 'w');
if fid_out == -1
    fclose(fid_in);
    error('Could not open output file.');
end

% Read the file line by line
lineNum = 0;
copying = 0;  % Counter to track copying state

while ~feof(fid_in)
    line = fgetl(fid_in);  % Read a line
    lineNum = lineNum + 1;
    
    % Check if the line contains the keyword
    if contains(line, keyword)
        copying = n + 1;  % Start copying (current line + n additional lines)
    end
    
    % Write to output file if copying is active
    if copying > 0
        fprintf(fid_out, '%s\n', line);
        copying = copying - 1;
    end
end

% Close files
fclose(fid_in);
fclose(fid_out);

disp('Processing complete. Output saved.');