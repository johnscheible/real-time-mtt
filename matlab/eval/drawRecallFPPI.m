function [ FPPI, recall ] = drawRecallFPPI( resdir, anno, opt ) % , imdir)
if nargin < 3
    opt = 0;
end
% 
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
[results, ts] = readTrackResults(resdir); %'../../standalone_tracker/results/seq02.11/'
if isempty(results)
    recall = [];
    FPPI = [];
    return 
end
%%%%%%%%%%%%%%%%%%%5
% imlist = dir('~/codes/eth_dataset/seq02-left/*.png');
% for i = 1:length(imlist)
%     imlist(i).fname = ['~/codes/eth_dataset/seq02-left/' imlist(i).name];
% end
%%%%%%%%%%%%%%%%%%%%%%%
FPPI = [];
recall = [];

for th = 0.01:0.02:1.01
    [TP, FP, FN, frames] = evalOneVideo(ts, results, anno, th, opt); % , imlist);
    recall(end+1) = TP / (TP + FN);
    FPPI(end + 1) = FP / frames;
end

% plot(FPPI, recall, '.-');
 grid on;

end
