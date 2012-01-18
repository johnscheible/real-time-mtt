function process_all_confidence(imdir, obank_dir, modelfile)
if nargin < 2
    obank_dir = './object_bank';
end
addpath(genpath(obank_dir));
load(modelfile);

% only upright standing person
model = getOneComponent(model, 2);

matlabpool open 8
files = dir([imdir '/*.png']);
parfor i = 1:length(files)
	filename = [imdir '/' files(i).name];
	conf_file = [imdir '/' files(i).name(1:end-4) '.conf'];

    disp(['process ' filename]);
	if(~exist(filename)) 
		disp(['file ' filename ' doesnot exist?\n'])
		continue;
	end
	if(exist(conf_file))
		continue;
	end
	% run LSVM detector
    im = imread(filename); 
    
    resizefactor = 2.0;
    im = imresize(double(im), resizefactor);
	
    [feat_py, scales] = featpyramid(im, 8, 10);
    Level=(model.interval+1):1:length(scales);
    [bbox, responsemap] = detect_with_responsemap(Level, feat_py, scales, im, model, model.thresh);
    
    top = reformDetections(bbox, resizefactor);
    conf = getConfidenceMap(responsemap, scales(Level), model, resizefactor);
    save_confidence(conf_file, top, conf);
end
matlabpool close

end

function conf_all = getConfidenceMap(responsemap, scales, model, resizefactor)
% convert the responsemap to confidence format
assert(length(scales) == length(responsemap));
% assuming  only one root filter
assert(length(model.rootfilters) == 1);
step= model.sbin; 
sz =  step .* model.rootfilters{1}.size;

padx = ceil(model.maxsize(2)/2+1);
pady = ceil(model.maxsize(1)/2+1);

for i = 1:length(scales)
    conf.size = sz(1) / resizefactor / scales(i);
    conf.size_ratio = sz(2) / sz(1);
    %%%%%% It is really important to know the correspondence!!!
    conf.minx = -padx * step / resizefactor / scales(i);
    conf.miny = -pady * step / resizefactor / scales(i);
    conf.step = step / resizefactor / scales(i);
    
    conf.map = responsemap{i};
    conf_all{i} = conf;
end
end

function [top] = reformDetections(bbox, resizefactor)

bbox = nms(bbox, 0.5);

top = bbox(:, [1:4, end - 1, end]);

top(:, 3:4) = top(:, 3:4) - top(:, 1:2) + 1;
[dummy, idx] = sort(top(:, end), 'descend');
top = top(idx, :);

top(:, 1:4) = top(:, 1:4) ./ resizefactor;
end

function model = getOneComponent(model, idx)

model.numcomponents = 1;
model.rootfilters = model.rootfilters(idx);
model.offsets = model.offsets(idx);

model.components = model.components(idx);
model.components{1}.rootindex = 1;
model.components{1}.offsetindex = 1;
end
