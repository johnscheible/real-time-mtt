#!/bin/sh
if [ $# -lt 1 ] 
then
	echo "Error - Number missing form command line argument"
	echo "Syntax : $0 number should be 1"
	exit 1
fi

POSTFIX=1
if [ $# -eq 2 ] 
then
	POSTFIX=$2
fi

SEQNAME=$1
rm -rf results/$SEQNAME.$POSTFIX
mkdir results/$SEQNAME.$POSTFIX

ROOTDIR=../sample/
IMLIST=$ROOTDIR"$SEQNAME"_imlist.txt
CONFLIST=$ROOTDIR"$SEQNAME"_conflist.txt
INIT_CAM="--init_cam_focal 500.0 --init_cam_x 0 --init_cam_y 1.0 --init_cam_z 0.0 --init_cam_v 1.0 --init_cam_yaw 0.0 --init_cam_horizon 215.0 --init_cam_xcenter 320"
PARAMS="--outdir results/$SEQNAME.$POSTFIX\
		--outvid results/$SEQNAME.$POSTFIX\
		--showimg true\
		--prob_stay 0.8 --prob_enter 0.15\
		--cam_motion_std_horizon 30 --cam_sample_std_horizon 10 --cam_motion_std_yaw 0.3 --cam_sample_std_yaw 0.05\
		--feat_sample_std_x 0.2	--feat_sample_std_z 0.2\
		--feat_sigma_u 4 --feat_sigma_v 4\
		--det_std_x 0.1 --det_std_y 0.15 --det_std_h 0.15
		--motion_std_x 0.5 --motion_std_y 0.1 --motion_std_z 0.5 --motion_std_vx 3.0 --motion_std_vz 3.0\
		--sample_std_x 0.5 --sample_std_y 0.04 --sample_std_z 0.5 --sample_std_vx 2.0 --sample_std_vz 2.0\
		--num_samples 20000 --num_burnin 4000 --num_thinning 400" # --motion_std_x 0.1 --motion_std_y 0.05 --motion_std_z 0.1 --sample_std_x 0.05 --sample_std_y 0.025 --sample_std_z 0.05"
		#--prob_move_stay 0.0 --prob_move_leave 0.0\

# gdb --args ./bin/ptrack --imglist $IMLIST --conflist $CONFLIST --rootdir $ROOTDIR --det_threshold 0.0 --det_weight 2.0 --dbg_show true $INIT_CAM $PARAMS
./bin/ptrack --imglist $IMLIST --conflist $CONFLIST --rootdir $ROOTDIR --det_threshold 0.0 --det_weight 2.0 --dbg_show true $INIT_CAM $PARAMS
