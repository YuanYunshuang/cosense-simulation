function [data] = read_info(varargin)
% READ_INFO Summary:
% Read trajectory information of all vehicles
%% Input: 
%        path: path to the info file
%    filename: name for saving trajectory data to current folder, 
%              defualt filename is 0 for not saving
%% Return:
%  trajectory: contrainers.Map
%              - key=frame(string)
%              - value=matrix(each row: vehicle_id, x,y,z,roll,pitch,yaw)
%% Code
    if nargin==0
        error('path to info should be given!')
    elseif nargin==1
        path = varargin{1};
        filename = 0;
    elseif nargin>1
        path = varargin{1};
        filenam = varargin{2};
    end

    fid = fopen(path);
    data = containers.Map;
    tline = fgetl(fid); % read the first line
    % data starts from the second line
    tline = fgetl(fid);
    while ischar(tline)
        line = textscan(tline, '%f', 'Delimiter',',');
        line = line{1};
        frame = line(1);
        table = reshape(line(2:end), [7, (length(line) - 1) / 7])';
        data(string(frame)) = table;
        tline = fgetl(fid);
    end  
    fclose (fid);
    if filename~=0
        save(name, 'data');
    end
end

