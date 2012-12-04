function [ FPPI, recall ] = recallFPPIDPMbboxes( anno, detdir, opt)
if nargin < 3
    opt = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%
show  = 0;
imlist = dir('~/codes/eth_dataset/seq02-left/*.png');
for i = 1:length(imlist)
    imlist(i).fname = ['~/codes/eth_dataset/seq02-left/' imlist(i).name];
end
%%%%%%%%%%%%%%%%%%%%%%%
detfiles = dir(fullfile(detdir, '*.mat'));
assert(length(imlist) == length(detfiles))
for i = 1:length(detfiles)
    if(i > length(anno) || isempty(anno(i).binfo) || ~anno(i).binfo)
        continue;
    end
    dets = load(fullfile(detdir, detfiles(i).name));
    top{i} = dets.bbox(dets.top, :);
% 	[top{i}] = load_confidence([detdir detfiles(i).name]);
end

recall = [];
FPPI = [];
addpath ../voc-release4.01/;

for th = -3:.2:3
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
		res = bbs(bbs(:, end) > th, 1:4)';
%         nidx = nms(res', 0.5);
%         res = res(:, nidx);
% 		res(3:4, :) = res(1:2, :) + res(3:4, :) - 1;
		% res = getBBs(ts(i), results, threshold);
		[e1, e2, e3] = get_one_frame_performance(gt, res, opt);
        
        if show 
            figure(200);
            imshow(imlist(i).fname);
            for j = 1:size(gt, 2)
                try
                    rectangle('position', [gt(1:2, j) gt(3:4, j) - gt(1:2, j)], 'linewidth', 2);
                catch
                    gt(1:4, j)'
                end
            end
            for j = 1:size(res, 2)
                rectangle('position', [res(1:2, j) res(3:4, j) - res(1:2, j)], 'linewidth', 2, 'edgecolor', 'r');
            end
            text(10, 10, ['TP ' num2str(e1) ' FP ' num2str(e2) ' FN ' num2str(e3)], 'BackgroundColor', 'w')
            pause
        end
		frames = frames + 1;
		TP = TP + e1; 
		FP = FP + e2; 
		FN = FN + e3; 
	end

    recall(end+1) = TP / (TP + FN);
    FPPI(end + 1) = FP / frames;
end

end
