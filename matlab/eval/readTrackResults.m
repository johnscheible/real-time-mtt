function [Tracks, ts] = readTrackResults(dirname)
files = dir([dirname '/target*']);
if(length(files) == 0) 
    Tracks = [];
    ts = [];
    return
end

for i = 1:length(files)
    idx = find_file(files, i-1);
	Tracks(i) = readTrack([dirname '/' files(idx).name]);
end

if(exist([dirname '/camera.txt']))
    ts = load([dirname '/camera.txt']);
    ts = ts(:, 1);
else
    ts = [];
end

end

function [idx] = find_file(files, tid)

for i = 1:length(files)
    if strcmp(files(i).name, ['target' num2str(tid) '.txt'])
        idx = i;
        return;
    end
end

asdasdafa

end
