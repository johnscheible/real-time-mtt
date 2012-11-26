function process_all_confidence_object(imdir, outdir, name)
ext = 'png';

obank_dir = './object_bank';
if ~exist(obank_dir, 'dir')
    system('wget http://vision.stanford.edu/projects/objectbank/MATLAB_release.zip; unzip MATLAB_release.zip -d object_bank; rm MATLAB_release.zip');
end
addpath(genpath(obank_dir));

%% load person model
%dpm_dir = 'voc-release3.1';
%if ~exist(dpm_dir, 'dir')
%    system('wget http://www.cs.brown.edu/~pff/latent-release3/voc-release3.1.tgz; tar xvf voc-release3.1.tgz; rm voc-release3.1.tgz');
%end
if ~strcmp(name, 'INRIA')
    data = load(['./vocmodel/VOC2008/' name '_final.mat']); %voc-release3.1/VOC2008/person_final.mat');
else
    data = load(['./vocmodel/INRIA/inria_final.mat']); %voc-release3.1/VOC2008/person_final.mat');
end

model = data.model;
clear data;
if ~exist(outdir, 'dir')
	mkdir(outdir);
end

matlabpool open
files = dir([imdir '/*.' ext]);
for idx = 1:16:length(files)
    parfor j = 1:16
        i = idx + j - 1;
        
        filename = [imdir '/' files(i).name];
        conf_file = [outdir '/' files(i).name(1:end-4) '.conf'];

        disp(['process ' filename]);

        if(~exist(filename)) 
            disp(['file ' filename ' doesnot exist?\n'])
            continue;
        end

        if(exist(conf_file))
            continue;
        end

        % run LSVM detector
        try
            im = imread(filename); 

            if (size(im, 1) < 500)
                resizefactor = 2.0;
            else
                resizefactor  = 1000 / size(im, 1);
            end

            im = imresize(double(im), resizefactor);

            [feat_py, scales] = featpyramid(im, 8, 10);
        %     Level = 1:model.interval:length(scales);
            Level=(model.interval+1):1:length(scales);
            [bbox, responsemap] = detect_with_responsemap(Level, feat_py, scales, im, model, model.thresh);

            top = reformDetections(bbox, resizefactor);
            conf = getConfidenceMap(responsemap, scales(Level), model, resizefactor);
            save_confidence(conf_file, top, conf);
        catch ee

        end
    end
end
matlabpool close

end

function conf_all = getConfidenceMap(responsemap, scales, model, resizefactor)
% convert the responsemap to confidence format
assert(length(scales) * length(model.rootfilters) == length(responsemap));
% assuming  only one root filter
% assert(length(model.rootfilters) == 1);
step= model.sbin; 
padx = ceil(model.maxsize(2)/2+1);
pady = ceil(model.maxsize(1)/2+1);
for i = 1:length(responsemap)
    type = mod(i - 1, length(model.rootfilters)) + 1;
    sid = floor((i - 1) / length(model.rootfilters)) + 1;
    
    sz =  step .* model.rootfilters{type}.size;
    
    conf.size = sz(1) / resizefactor / scales(sid);
    
    conf.size_ratio = sz(2) / sz(1);
    %%%%%% It is really important to know the correspondence!!!
    conf.step = step / resizefactor / scales(sid);
    conf.minx = -padx * conf.step;
    conf.miny = -pady * conf.step;
    
    conf.map = responsemap{i};
    conf_all{i} = conf;
end
end

function [top] = reformDetections(bbox, resizefactor)

bbox = nms(bbox, 0.5);
if(size(bbox, 1) == 0)
    top = zeros(0, 6);
    return;
end

top = bbox(:, [1:4, end - 1, end]);

top(:, 3:4) = top(:, 3:4) - top(:, 1:2) + 1;
[dummy, idx] = sort(top(:, end), 'descend');
top = top(idx, :);

top(:, 1:4) = top(:, 1:4) ./ resizefactor;
end
