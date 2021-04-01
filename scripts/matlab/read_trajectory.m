path = "\\koko\tmp\yunshuang\data\simulation\j509e532\info.csv";
data = containers.Map;
fid = fopen(path);
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
save('trajectory_j509_e532.mat', 'data');
