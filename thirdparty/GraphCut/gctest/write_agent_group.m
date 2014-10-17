data = load('log_0613_1218.txt');
data(1:113, 4:6) = data(1:113, 1:3)
data(1:113, 6) = data(1:113, 6) - 80;
data(114:end, 4:6) = data(114:end, 1:3)
data(114:end, 6) = data(114:end, 6) + 80;

fid = fopen('agent_group.txt', 'w');
for i=1:size(data, 1)
    fprintf(fid, '%f %f %f %f %f %f %d\n', data(i, 1), data(i, 2), data(i, 3), data(i, 4), data(i, 5), data(i, 6), data(i, 7));
end
fclose(fid);