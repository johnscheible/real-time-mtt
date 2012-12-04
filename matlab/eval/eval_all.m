clear

seqname = 'seq02';
%%
ids = 21:24; % [1:4 21:24 31:34 41:44 51:54 61:64 71:74 81:82];
%%
annofile = ['~/codes/eth_dataset/' seqname '-left/annotation.txt'];
anno = read_annotation(annofile); %'~/codes/eth_dataset/seq02-left/linthescher-annot.idl'
matlabpool open 4
parfor a = 1:length(ids)
    i = ids(a);
    [ eval(a).name] = [seqname 'dets' num2str(i)]; 
    [ eval(a).fppi, eval(a).recall] = drawRecallFPPI(['../../standalone_tracker/results/' seqname '.' num2str(i+9) '/'], anno);
end
%%
parfor a = 1:length(ids)
    i = ids(a);
    [ temp(a).name] = [seqname 'track' num2str(i)];
	[ temp(a).fppi,  temp(a).recall] = evalTrackDetections(['../../standalone_tracker/results/' seqname '.' num2str(i + 9) '/'], anno, 0);	
end
eval = [eval, temp];
matlabpool close
%%
[ eval(end+1).name ] = 'detector'; 
[ eval(end).fppi, eval(end).recall ] = recallFPPIdetector(anno, ['~/codes/eth_dataset/' seqname '-left/']);

[ eval(end+1).name ] = 'VOC2006'; 
[ eval(end).fppi, eval(end).recall ] = recallFPPIDPMbboxes( anno, ['~/codes/eth_dataset/dets/' eval(end).name]);

[ eval(end+1).name ] = 'VOC2007'; 
[ eval(end).fppi, eval(end).recall ] = recallFPPIDPMbboxes( anno, ['~/codes/eth_dataset/dets/' eval(end).name]);

[ eval(end+1).name ] = 'VOC2009'; 
[ eval(end).fppi, eval(end).recall ] = recallFPPIDPMbboxes( anno, ['~/codes/eth_dataset/dets/' eval(end).name]);

save([seqname 'evals3'], 'eval');
%%
annofile = '~/codes/eth_dataset/wojek_annotations/eth-linthescher-extended.idl';
anno = readIDLFile(annofile, 930);

matlabpool open 4
parfor a = 1:length(ids)
% for a = 1:length(ids)
    i = ids(a);
    [ eval2(a).name] = [seqname 'dets' num2str(i)]; 
    [ eval2(a).fppi, eval2(a).recall] = drawRecallFPPI(['../../standalone_tracker/results/' seqname '.' num2str(i+9) '/'], anno, 1);
end
%%
parfor a = 1:length(ids)
    i = ids(a);
    [ temp(a).name] = [seqname 'track' num2str(i)];
	[ temp(a).fppi,  temp(a).recall] = evalTrackDetections(['../../standalone_tracker/results/' seqname '.' num2str(i + 9) '/'], anno, 1);	
end
eval2 = [eval2, temp];
matlabpool close
%%
[ eval2(end+1).name ] = 'detector'; 
[ eval2(end).fppi, eval2(end).recall ] = recallFPPIdetector( anno, ['~/codes/eth_dataset/' seqname '-left/'], 1);

[ eval2(end+1).name ] = 'VOC2006'; 
[ eval2(end).fppi, eval2(end).recall ] = recallFPPIDPMbboxes( anno, ['~/codes/eth_dataset/dets/' eval2(end).name], 1);

[ eval2(end+1).name ] = 'VOC2007'; 
[ eval2(end).fppi, eval2(end).recall ] = recallFPPIDPMbboxes( anno, ['~/codes/eth_dataset/dets/' eval2(end).name], 1);

[ eval2(end+1).name ] = 'VOC2009'; 
[ eval2(end).fppi, eval2(end).recall ] = recallFPPIDPMbboxes( anno, ['~/codes/eth_dataset/dets/' eval2(end).name], 1);
save([seqname 'evals4'], 'eval2');

return;

%%
figure;
col = colormap;

hold on; 
% texts = {};
% for a = 1:length(ids)
%     texts{end + 1} = eval(a).name;
%     idx = mod(ids(a)*2, 64) + 1;
%     plot(eval(a).fppi, eval(a).recall, 'color', col(idx, :));
% %     plot(eval(a).fppi, eval(a).recall, 'color', col(idx, :));
% %     scatter(eval(a).fppi, eval(a).recall, 'rx');%, 'facecolor', col(idx, :));
%     drawnow;
% end; 

texts = {}; a = 1;
texts{end + 1} = 'Proposed Method';
plot(eval(a).fppi, eval(a).recall, 'b-', 'LineWidth', 3);

idx = 1;
texts{end+1} = 'Felzenszwalb DPM';
plot(eval(end).fppi, eval(end).recall, 'c--', 'LineWidth', 3);

hold off 

axis([0 1.5 0 0.8])
legend(texts);

grid on
title([seqname ' experiments'])
xlabel('#false positive per image')
ylabel('Recall')
