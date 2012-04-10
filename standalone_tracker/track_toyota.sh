#!/bin/sh
if [ $# -lt 2 ] 
then
	echo "Error - Number missing form command line argument"
	echo "Syntax : $0 number should be larger than 2"
	exit 1
fi
OBJ=$1
POSTFIX=$2

IFRAME=0
EFRAME=1000000
if [ $# -ge 3 ] 
then
	IFRAME=$3
	POSTFIX=$POSTFIX.$IFRAME
fi
if [ $# -ge 4 ] 
then
	EFRAME=$4
	POSTFIX=$POSTFIX.$EFRAME
fi

HOR=1030
if [ $# -ge 5 ] 
then
	HOR=$5
fi

SEQNAME=Toyota
ROOTDIR=/home/wgchoi/ToyotaData/

IMLIST=$ROOTDIR"imlist.txt"
VPLIST=$ROOTDIR"vplist.txt"
if [ $OBJ -eq 0 ]; then
	CONFLIST=$ROOTDIR"inria_conflist.txt"
	OBJ_PARAMS="--objtype Person\
				--min_height 1.3 --max_height 2.3 --det_threshold 0.0 --det_weight 2.0"
	POSTFIX=person.$POSTFIX
else
	CONFLIST=$ROOTDIR"car_conflist.txt"
	OBJ_PARAMS="--objtype Car\
				--min_height 0.8 --max_height 1.8 --det_threshold -0.5 --det_weight 2.0"
	POSTFIX=car.$POSTFIX
fi
rm -rf results/$SEQNAME.$POSTFIX
mkdir results/$SEQNAME.$POSTFIX
INIT_CAM="--init_cam_focal 1500.0 --init_cam_x 0 --init_cam_y 1.7 --init_cam_z 0.0 --init_cam_v 1.0 --init_cam_yaw 0.0 --init_cam_horizon $HOR --init_cam_xcenter 1020"
PARAMS="--outdir results/$SEQNAME.$POSTFIX\
		--outvid results/$SEQNAME.$POSTFIX\
		--showimg false\
		--mean_horizon 1000 --std_horizon 50
		--prob_stay 0.8 --prob_enter 0.15\
		--cam_motion_std_horizon 100 --cam_sample_std_horizon 10 --cam_motion_std_yaw 0.6 --cam_sample_std_yaw 0.05 --cam_motion_std_v 5 --cam_sample_std_v 1\
		--feat_sample_std_x 0.2	--feat_sample_std_z 0.2\
		--feat_sigma_u 20 --feat_sigma_v 20\
		--det_std_x 0.1 --det_std_y 0.15 --det_std_h 0.15
		--motion_std_x 0.5 --motion_std_y 0.1 --motion_std_z 0.5 --motion_std_vx 3.0 --motion_std_vz 3.0\
		--sample_std_x 0.5 --sample_std_y 0.04 --sample_std_z 0.5 --sample_std_vx 2.0 --sample_std_vz 2.0\
		--num_samples 10000 --num_burnin 2000 --num_thinning 200" # --motion_std_x 0.1 --motion_std_y 0.05 --motion_std_z 0.1 --sample_std_x 0.05 --sample_std_y 0.025 --sample_std_z 0.05"
# gdb --args ./bin/ptrack --imglist $IMLIST --conflist $CONFLIST --vplist $VPLIST --rootdir $ROOTDIR --dbg_show true $INIT_CAM --iframe $IFRAME --eframe $EFRAME $PARAMS $OBJ_PARAMS
./bin/ptrack --imglist $IMLIST --conflist $CONFLIST --vplist $VPLIST --rootdir $ROOTDIR --dbg_show true $INIT_CAM --iframe $IFRAME --eframe $EFRAME $PARAMS $OBJ_PARAMS
