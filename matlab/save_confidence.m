function save_confidence(filename, top, conf)
fp = fopen(filename, 'w');

fwrite(fp,'CON2','char');

num_dets = size(top, 1);
fwrite(fp, num_dets, 'uint');
for i = 1:size(top, 1)
    fwrite(fp, single(top(i, [1:4 end 5])), 'single');
end

num_conf = length(conf);
fwrite(fp, num_conf, 'uint');
for i = 1:num_conf
    % size
    fwrite(fp, single(conf{i}.size), 'single');
    % size_ratio
    fwrite(fp, single(conf{i}.size_ratio), 'single');
    % step
    fwrite(fp, single(conf{i}.step), 'single');
    % minx
    fwrite(fp, single(conf{i}.minx), 'single');
    % miny
    fwrite(fp, single(conf{i}.miny), 'single');
    % map size
    map_size = size(conf{i}.map);
    fwrite(fp, single(map_size), 'single');
    % write map
    fwrite(fp, single(conf{i}.map), 'single');
end

fclose(fp);

end