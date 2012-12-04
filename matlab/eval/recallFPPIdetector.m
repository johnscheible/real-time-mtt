function [ FPPI, recall ] = recallFPPIdetector( anno, detdir, opt)%, fbase)
if nargin < 3
    opt = 0;
end
% if opt == 0
%     anno = read_annotation(annofile); %'~/codes/eth_dataset/seq02-left/linthescher-annot.idl'
% else
%     ids = readIDL(annofile);
%     anno = struct('fname', {}, 'rts', {}, 'binfo', {});
%     
%     for i = 1:length(ids)
%         fnum = sscanf(ids(i).img, 'left/image_%08d_0.png', 1) - fbase + 1;
%         anno(fnum).fname = ids(i).img;
%         anno(fnum).rts = ids(i).bb;
%         anno(fnum).binfo = 1;
%     end
% end

detfiles = dir([detdir '*.conf']);
for i = 1:length(detfiles)
	[top{i}] = load_confidence([detdir detfiles(i).name]);
end

recall = [];
FPPI = [];

for th = -1:.1:3
	TP = 0;
	FP = 0;
	FN = 0;
	frames = 0;
	for i = 1:length(anno)
		if(isempty(anno(i).binfo) || ~anno(i).binfo)
			continue;
		end
		gt = anno(i).rts';

		bbs = top{i};
		res = bbs(bbs(:, 5) > th, 1:4)';

		res(3:4, :) = res(1:2, :) + res(3:4, :) - 1;
		% res = getBBs(ts(i), results, threshold);
		[e1, e2, e3] = get_one_frame_performance(gt, res, opt);
		frames = frames + 1;
		TP = TP + e1; 
		FP = FP + e2; 
		FN = FN + e3; 
	end

    recall(end+1) = TP / (TP + FN);
    FPPI(end + 1) = FP / frames;
end

end
