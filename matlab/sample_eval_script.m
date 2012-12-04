clear 

addpath ./eval
addpath ./IDL

imgformat = '%010d.png';
basenum = 0 - 1;

% datadir = '/home/wgchoi/datasets/necdata/0104/2011_09_26_drive_0104/image_02/dets/';
% resdir = '/home/wgchoi/codes/ped_tracker_google/trunk/standalone_tracker/results/0104.car.test_w4.car/';
% annofile = '/home/wgchoi/datasets/necdata/gtbbox/0104_image02.idl';
datadir = '/home/wgchoi/datasets/necdata/0093/2011_09_26_drive_0093/image_02/dets/';
resdir = '/home/wgchoi/codes/ped_tracker_google/trunk/standalone_tracker/results/0093.car.org/';
annofile = '/home/wgchoi/datasets/necdata/gtbbox/0093_image02.idl';
% datadir = '/home/wgchoi/datasets/necdata/0056/2011_09_26_drive_0056/image_02/dets/';
% resdir = '/home/wgchoi/codes/ped_tracker_google/trunk/standalone_tracker/results/0056.car.org/';
% annofile = '/home/wgchoi/datasets/necdata/gtbbox/0056_image02.idl';
%%
% read annotations in idl format 
anno = readIDL(annofile);
% corret the order of x1, x2, y1, y2 (increasing)
anno = correctIDL(anno);

% read detection results and convert it into IDL format
detidl = confs2idl(datadir, '');
trackidl  = tracks2idl(resdir, imgformat, basenum); 
%%
% compute fppi/recall
clf;

[ FPPI, recall, thlist ] = recallfppi_idls( anno , trackidl, 1, 1);
draw_fppimr(recall, FPPI, 'b', '.-')
[ FPPI, recall, thlist ] = recallfppi_idls( anno , detidl, 1, 1);
hold on;
draw_fppimr(recall, FPPI, 'r', '-')
hold off;
legend({'track' 'dpm'})