function idl = confs2idl(detdir, imgprefix)

%%%%%%%%%%%%%%%%%%%%%%%
detfiles = dir(fullfile(detdir, '*.conf'));

idl = struct('bb', cell(length(detfiles), 1), 'img', cell(length(detfiles), 1), 'score', cell(length(detfiles), 1));
for i = 1:length(detfiles)
%     dets = load(fullfile(detdir, detfiles(i).name));     
	[top] = load_confidence([detdir detfiles(i).name]);
    %idl(i).bb = convert_width([top(:, 1:2) top(:, 1:2) + top(:, 3:4) - 1], 2.3);
    idl(i).bb = [top(:, 1:2) top(:, 1:2) + top(:, 3:4) - 1];
    idl(i).score = top(:, 5);
    
    idx = find(detfiles(i).name == '.', 1, 'last');
    idl(i).img = [imgprefix, detfiles(i).name(1:idx) 'png'];
end

end

function bbs = convert_width(bbs, hw_ratio)

h = bbs(:, 4) - bbs(:, 2) + 1;
w = bbs(:, 3) - bbs(:, 1) + 1;

x = bbs(:, 1) + w ./ 2;
w = h ./ hw_ratio;

bbs(:, 1) = x - w ./ 2;
bbs(:, 3) = bbs(:, 1) + w;

end
