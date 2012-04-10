function test(idx)
if ischar(idx)
	idx = str2num(idx);
end

imgdir = '~/ToyotaData/left_images';
outdir = '~/ToyotaData/vpvotes';
if ~exist(outdir, 'dir')
	mkdir(outdir);
end

vstep = 5;
mina = 0.1 * pi;
[imfiles] = dir(fullfile(imgdir, '*.jpg'));

list = ((idx - 1) * 100 + 1):min(length(imfiles), (idx * 100));

for i = list % 1:length(imfiles)
	try
		imfile = imfiles(i).name;
		disp(['begin ' imfile]);

		im=imread(fullfile(imgdir, imfiles(i).name));
		im = double(rgb2gray(im));

		resizefactor = 1.0;
		if(size(im, 1) > 600)
			resizefactor = 600 / size(im, 1);
		end
		im = imresize(im, resizefactor);

		imsz = size(im);
		lines = APPgetLargeConnectedEdges(im, 30);
		lidx = find((lines(:, 5) >= mina & lines(:, 5) <= (pi - mina)) | ...
					(lines(:, 5) <= -mina & lines(:, 5) >= (mina - pi)) );
		vote = heuristic_vote(lines(lidx, :), [imsz(2), imsz(1)], vstep);

% 		save(fullfile(outdir, ['votemap' num2str(i, '%06d') '.mat']) , 'vote', 'lines', 'lidx', 'imsz', 'resizefactor', 'vstep', ...
% 																'imgdir', 'imfile');
		disp(['done ' imfile]);
	catch ee
		ee
	end
end
