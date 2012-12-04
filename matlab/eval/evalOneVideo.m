function [TP, FP, FN, frames] = evalOneVideo(ts, results, anno, threshold, opt, imlist)
frames = 0;
TP = 0;
FP = 0;
FN = 0;
% % read annotations
% anno = read_annotation(annofile);
% if(length(anno) == 0)
% 	return;
% end
% % read results
% [results, ts] = readTrackResults(resdir);

for i = 1:length(ts)
	if(i > length(anno) || isempty(anno(i).binfo) || ~anno(i).binfo)
		continue;
	end

	gt = anno(i).rts';
	res = getBBs(ts(i), results, threshold);
	res = convert_width(res, 2.3); % best among 3, 2, 2.5, 2.25, 2.3, 2.4
	% gt = convert_width(gt, 2); % best but unfair???
	[e1, e2, e3] = get_one_frame_performance(gt, res, opt);

    if nargin > 5
        if (opt == 0)
            gt = filter_bbs(gt, 60);
            res = filter_bbs(res, 60);
        end
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
end


function [bbs, stamps] = getBBs(ts, tracks, conf)
bbs = zeros(4, 0);
stamps = zeros(1, 0);
idx = zeros(1, 0);

for i = 1:length(tracks)
	minidx = 0;
	mindist = 0.03;

	for j = 1:length(tracks(i).ts)
		dist = tracks(i).ts(j) - ts;
		if(abs(dist) < mindist)
			minidx = j;
			mindist = dist;
		end
	end

	% found
	if minidx > 0
		if(conf >= -10000)
			if(tracks(i).conf(minidx) < conf)
				continue;
			end
		end
		% prune out the boxes outside of the image
		%if(inImage(640, 480, tracks(i).rect(:, minidx)))
		bbs = [bbs, [tracks(i).rect(1:2, minidx); tracks(i).rect(1:2, minidx) + tracks(i).rect(3:4, minidx)]];
		stamps(end + 1) = tracks(i).ts(minidx);
		idx(end + 1) = i;
		%end
	end
end

end

function rts = convert_width(rts, hw_ratio)

h = rts(4, :) - rts(2, :) + 1;
w = rts(3, :) - rts(1, :) + 1;

x = rts(1, :) + w ./ 2;
w = h ./ hw_ratio;

rts(1, :) = x - w ./ 2;
rts(3, :) = rts(1, :) + w;

end
