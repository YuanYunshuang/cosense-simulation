function [frame, timestamp, tform, other] = read_meta(path)
% READ_INFO Summary:
% Read trajectory information of all vehicles
%% Input: 
%        path: path to the info file
%% Return:
%        data: contrainers.Map
%              - key=frame(string)
%              - value=matrix(each row: vehicle_id, x,y,z,roll,pitch,yaw)
%% Code

    fid = fopen(path);
    data = containers.Map;
    line1 = fgetl(fid);
    line2 = fgetl(fid);
    line3 = fgetl(fid);
    if ischar(line3)
        % read lidar meta file
        line1 = textscan(line1, '%f', 'Delimiter',',');
        line1 = line1{1};
        frame = line1(1);
        timestamp = line1(2);
        line2 = textscan(line2, '%f', 'Delimiter',',');
        line2 = line2{1};
        rot = line2(1:3) / 180 * pi;
        loc = line2(4:6);
        loc(2) = -loc(2);
        rotmat = eul2rotm(rot', "XYZ");
        tform = rigid3d(rotmat, loc');
        line3 = textscan(line3, '%f', 'Delimiter',',');
        other = line3{1};
    else
        % read image meta file
        line1 = textscan(line1, '%f', 'Delimiter',',');
        line1 = line1{1};
        frame = line1(1);
        timestamp = line1(2);
        line2 = textscan(line2, '%f', 'Delimiter',',');
        line2 = line2{1};
        rot = line2(1:3) / 180 * pi;
        loc = line2(4:6);
        rotmat = eul2rotm(rot', "XYZ");
        tform = rigid3d(rotmat, loc);
        other = -1;
    end  
    fclose (fid);
end

