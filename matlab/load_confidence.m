function [top, conf] = load_confidence(filename)
fp = fopen(filename, 'r');

header = fread(fp, 4, 'char');
if (header(1) ~= 'C' && ...
    header(2) ~= 'O' && ...
    header(3) ~= 'N' && ...
    header(4) ~= 'F')
    fprintf('invalid header %s\n', header);
end

top = [];
num_dets = fread(fp, 1, 'uint');
for i = 1:num_dets
    top(i, :) = fread(fp, 6, 'single');
end

if nargout < 2
	fclose(fp);
	conf = {};
	return;
end

num_conf = fread(fp, 1, 'uint');
for i = 1:num_conf
    % size
    conf{i}.size = fread(fp, 1, 'single');
    % size_ratio
    conf{i}.size_ratio = fread(fp, 1, 'single');
    % step
    conf{i}.step = fread(fp, 1, 'single');
    % minx
    conf{i}.minx = fread(fp, 1, 'single');
    % miny
    conf{i}.miny = fread(fp, 1, 'single');
    % map size
    map_size = fread(fp, 2, 'single')';
    % write map
    conf{i}.map = fread(fp, map_size, 'single');
end

fclose(fp);

end
