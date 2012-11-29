#!/bin/sh
if [ $# -lt 2 ] 
then
	echo "Error - Number missing form command line argument"
	echo "Syntax : $0 number should be larger than 2"
	exit 1
fi
OBJ=$1
POSTFIX=$2

SEQNAME=$3
IFRAME=0
EFRAME=1000000

HOR=170
if [ $# -ge 4 ] 
then
	HOR=$5
fi

ROOTDIR=/home/wgchoi/datasets/necdata/

IMLIST=$ROOTDIR$SEQNAME"_imlist.txt"
#VPLIST=$ROOTDIR$SEQNAME"vplist.txt"
if [ $OBJ -eq 0 ]; then
	CONFLIST=$ROOTDIR$SEQNAME"_inria_conflist.txt"
	OBJ_PARAMS="--objtype Person\
				--min_height 1.3 --max_height 2.3 --det_threshold 0.0 --det_weight 2.0"
	POSTFIX=person.$POSTFIX
else
	CONFLIST=$ROOTDIR$SEQNAME"_car_conflist.txt"
	OBJ_PARAMS="--objtype Car\
				--min_height 0.8 --max_height 1.8 --det_threshold -0.5 --det_weight 1.0"
	POSTFIX=car.$POSTFIX
fi

rm -rf results/$SEQNAME.$POSTFIX
mkdir results/$SEQNAME.$POSTFIX
INIT_CAM="--init_cam_focal 800.0 --init_cam_x 0 --init_cam_y 1.3 --init_cam_z 0.0 --init_cam_v 5.0 --init_cam_yaw 0.0 --init_cam_horizon $HOR --init_cam_xcenter 621"
PARAMS="--outdir results/$SEQNAME.$POSTFIX\
		--outvid results/$SEQNAME.$POSTFIX\
		--showimg true\
		--mean_horizon 170 --std_horizon 10
		--prob_stay 0.9 --prob_enter 0.2\
		--cam_motion_std_horizon 100 --cam_sample_std_horizon 10 --cam_motion_std_yaw 0.6 --cam_sample_std_yaw 0.05 --cam_motion_std_v 5 --cam_sample_std_v 1\
		--feat_sample_std_x 0.2	--feat_sample_std_z 0.2\
		--feat_sigma_u 5 --feat_sigma_v 20\
		--det_std_x 0.1 --det_std_y 0.15 --det_std_h 0.15
		--motion_std_x 0.5 --motion_std_y 0.1 --motion_std_z 0.5 --motion_std_vx 3.0 --motion_std_vz 3.0\
		--sample_std_x 0.5 --sample_std_y 0.04 --sample_std_z 0.5 --sample_std_vx 2.0 --sample_std_vz 2.0\
		--num_samples 10000 --num_burnin 2000 --num_thinning 200" # --motion_std_x 0.1 --motion_std_y 0.05 --motion_std_z 0.1 --sample_std_x 0.05 --sample_std_y 0.025 --sample_std_z 0.05"
# gdb --args ./bin/ptrack --imglist $IMLIST --conflist $CONFLIST --rootdir $ROOTDIR --dbg_show true $INIT_CAM --iframe $IFRAME --eframe $EFRAME $PARAMS $OBJ_PARAMS
./bin/ptrack --imglist $IMLIST --conflist $CONFLIST --rootdir $ROOTDIR --dbg_show true $INIT_CAM --iframe $IFRAME --eframe $EFRAME $PARAMS $OBJ_PARAMS
