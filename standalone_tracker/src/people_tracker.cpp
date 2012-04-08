#include <stdio.h>
#include <iostream>
#include <stdint.h>
// observation
#include <observation/ObservationNode.h>
#include <observation/DetectionReadinNode.h>
#include <observation/FaceNode.h>
#include <observation/ObservationManager.h>
// tracker
#include <tracker/rjmcmc_tracker.h>
#include <tracker/target_manager.h>
// temp
#include <common/util.h>
#include <common/FeatTracker.h>
// program option
#include <boost/program_options.hpp>

using namespace std;
using namespace people;
namespace po = boost::program_options;

typedef struct _param_set {
	// input data
	std::string root_dir;
	std::string im_list_file;
	std::string conf_list_file;
	std::string vp_list_file;
	std::string	out_dir;
	std::string	out_vid;

	bool		showimg;
	bool		cam_estimate;
	bool		interaction_on;
	
	int 		iframe;
	int 		eframe;

	double		fps;
	// observation paramters
	double 		min_height;
	double 		max_height;
	double 		feat_sigma_u;
	double 		feat_sigma_v;
	double 		det_weight;
	double 		det_threshold;
	double 		det_scale;
	// mcmc tracker related parameters
	bool 		dbg_show;
	int 		num_samples;
	int			num_burnin;
	int			num_thinning;
	int 		min_samples;
	int			max_feats;

	double		init_cam_focal;
	double		init_cam_x;
	double		init_cam_y;
	double		init_cam_z;
	double		init_cam_v;
	double		init_cam_yaw;
	double		init_cam_horizon;
	double		init_cam_xcenter;

	double		motion_std_x;
	double		motion_std_y;
	double		motion_std_z;
	double		motion_std_vx;
	double		motion_std_vz;

	double		repulsion_const;
	double		group_const;
	double 		group_slope;
	double		group_threshold;

	double		sample_std_x;
	double		sample_std_y;
	double		sample_std_z;
	double		sample_std_vx;
	double		sample_std_vz;

	double		cam_motion_std_x;
	double		cam_motion_std_z;
	double		cam_motion_std_v;
	double		cam_motion_std_yaw;
	double		cam_motion_std_horizon;
	
	double		cam_sample_std_x;
	double		cam_sample_std_z;
	double		cam_sample_std_v;
	double		cam_sample_std_yaw;
	double		cam_sample_std_horizon;
	
	double		feat_sample_std_x;
	double		feat_sample_std_z;

	double		prob_enter;
	double		prob_stay;
	double		prob_feat_enter;
	double		prob_feat_stay;

	double		prob_move_add;
	double		prob_move_delete;
	double		prob_move_stay;
	double		prob_move_leave;
	double		prob_move_update;
	double		prob_move_feat_stay;
	double		prob_move_feat_leave;
	double		prob_move_feat_update;
	double		prob_move_cam_update;
	double		prob_move_interaction_flip;

	double		detection_std_x;
	double		detection_std_y;
	double		detection_std_h;
}param_set;

param_set parse_arguments(int ac, char **av)
{
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("rootdir", po::value<std::string>(), "root directory of data")
		("imglist", po::value<std::string>(), "image input list file (required)")
		("conflist", po::value<std::string>(), "confidence input list file (required)")
		("vplist", po::value<std::string>(), "vanishing point estimate file (optional)")
		("outdir", po::value<std::string>(), "results directory")
		("outvid", po::value<std::string>(), "output video")
		("showimg", po::value<bool>(), "show results")
		("cam_estimate", po::value<bool>(), "estimate camera motion")
		("interaction_on", po::value<bool>(), "use interaction model")
		("fps", po::value<double>(), "FPS (15)")
		("iframe", po::value<int>(), "initial frame (0, optional)")
		("eframe", po::value<int>(), "last frame (inf, optional)")
		// observation parameters
		("min_height", po::value<double>(), "minimum height of a person (m) (1.2)")
		("max_height", po::value<double>(), "maximum height of a person (m) (2.2)")
		("feat_sigma_u", po::value<double>(), "feature observation sigma u (2)")
		("feat_sigma_v", po::value<double>(), "feature observation sigma v (2)")
		("det_weight", po::value<double>(), "detector weight (1.0)")
		("det_scale", po::value<double>(), "detector scale (1.0718)")
		("det_threshold", po::value<double>(), "detector threshold (0.0)")
		("det_std_x", po::value<double>(), "detector std x (0.05 * deth)")
		("det_std_y", po::value<double>(), "detector std y (0.1 * deth)")
		("det_std_h", po::value<double>(), "detector std h (0.1 * deth)")
		// tracker paramters
		("dbg_show", po::value<bool>(), "debug prints (false)")
		("num_samples", po::value<int>(), "number of samples in MCMC (5000)")
		("num_burnin", po::value<int>(), "number of samples to burn-in (1000)")
		("num_thinning", po::value<int>(), "number of samples to be ignored (100)")
		("min_samples", po::value<int>(), "minimum number of samples to keep a track (5)")
		("max_feats", po::value<int>(), "maximum number of features to be used in one frame (40)")
		// initial camera
		("init_cam_focal", po::value<double>(), "initial camera parameter focal (required)")
		("init_cam_x", po::value<double>(), "initial camera parameter x (required)")
		("init_cam_y", po::value<double>(), "initial camera parameter y (required)")
		("init_cam_z", po::value<double>(), "initial camera parameter z (required)")
		("init_cam_v", po::value<double>(), "initial camera parameter v (required)")
		("init_cam_yaw", po::value<double>(), "initial camera parameter yaw (required)")
		("init_cam_horizon", po::value<double>(), "initial camera parameter horizon (required)")
		("init_cam_xcenter", po::value<double>(), "initial camera parameter xcenter (required)")
		// person motion parameters
		("motion_std_x", po::value<double>(), "person motion std in x (0.5 / sec)")
		("motion_std_y", po::value<double>(), "person motion std in y (0.1 / sec)")
		("motion_std_z", po::value<double>(), "person motion std in z (0.5 / sec)")
		("motion_std_vx", po::value<double>(), "person motion std in x velocity (3.0 / sec)")
		("motion_std_vz", po::value<double>(), "person motion std in z velocity (3.0 / sec)")
		// interaction parameters
		("repulsion_const", po::value<double>(), "")
		("group_const", po::value<double>(), "")
		("group_slope", po::value<double>(), "")
		("group_threshold", po::value<double>(), "")
		// person sample parameters
		("sample_std_x", po::value<double>(), "person sample std in x (0.2 / sec)")
		("sample_std_y", po::value<double>(), "person sample std in y (0.04 / sec)")
		("sample_std_z", po::value<double>(), "person sample std in z (0.2 / sec)")
		("sample_std_vx", po::value<double>(), "person sample std in x velocity (1.0 / sec)")
		("sample_std_vz", po::value<double>(), "person sample std in z velocity (1.0 / sec)")
		// camera motion parameters
		("cam_motion_std_x", po::value<double>(), "camera motion std in x (0.5 / sec)")
		("cam_motion_std_z", po::value<double>(), "camera motion std in z (0.5 / sec)")
		("cam_motion_std_v", po::value<double>(), "camera motion std in velocity (3.0 / sec)")
		("cam_motion_std_yaw", po::value<double>(), "camera motion std in yaw (0.1 rad/ sec)")
		("cam_motion_std_horizon", po::value<double>(), "camera motion std in horizon (30 / sec)")
		// camera sample parameters
		("cam_sample_std_x", po::value<double>(), "camera sample std in x (0.2 / sec)")
		("cam_sample_std_z", po::value<double>(), "camera sample std in z (0.2 / sec)")
		("cam_sample_std_v", po::value<double>(), "camera sample std in velocity (1.0 / sec)")
		("cam_sample_std_yaw", po::value<double>(), "camera sample std in yaw (0.05 rad/ sec)")
		("cam_sample_std_horizon", po::value<double>(), "camera sample std in horizon (10 / sec)")
		// feat sample paramters
		("feat_sample_std_x", po::value<double>(), "feature sample std in x (0.03)")
		("feat_sample_std_z", po::value<double>(), "feature sample std in z (0.03)")
		// prob enter/leabe
		("prob_enter", po::value<double>(), "probability of observing new person (0.2)")
		("prob_stay", po::value<double>(), "probability for a person to stay (0.8)")
		("prob_feat_enter", po::value<double>(), "probability of observing new feature (0.2)")
		("prob_feat_stay", po::value<double>(), "probability for a feature to stay (0.9)")
		// move proposal
		("prob_move_add", po::value<double>(), "proposal add a target (0.025)")
		("prob_move_delete", po::value<double>(), "proposal delete a target (0.025)")
		("prob_move_stay", po::value<double>(), "proposal stay a target (0.05)")
		("prob_move_leave", po::value<double>(), "proposal leave a target (0.05)")
		("prob_move_update", po::value<double>(), "proposal update a target's state (0.2)")
		("prob_move_feat_stay", po::value<double>(), "proposal stay a feature (0.075)")
		("prob_move_feat_leave", po::value<double>(), "proposal leave a feature (0.075)")
		("prob_move_cam_update", po::value<double>(), "proposal update camera parameters (0.3)")
		("prob_move_interaction_flip", po::value<double>(), "proposal flip an interaction (0.1)")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(ac, av, desc), vm);
	po::notify(vm);    

	if (vm.count("help")) {
		cout << desc << "\n";
		exit(1);
	}

	param_set ret;
	//////////////////////////////////////////////////////////////////
	// input data
	//////////////////////////////////////////////////////////////////
	if(!vm.count("rootdir"))  {
		ret.root_dir = "/home/wgchoi/codes/eth_dataset/";
	}
	else {
		ret.root_dir = vm["rootdir"].as<std::string>();
	}
	if(!vm.count("imglist"))  {
		std::cout << "imglist must be provided." << std::endl;
		exit(1);
	}
	else {
		ret.im_list_file = vm["imglist"].as<std::string>();
	}

	if(!vm.count("conflist"))  {
		std::cout << "conflist must be provided." << std::endl;
		exit(1);
	}
	else {
		ret.conf_list_file = vm["conflist"].as<std::string>();
	}

	if(!vm.count("vplist"))  {
		ret.vp_list_file = "";
	}
	else {
		ret.vp_list_file = vm["vplist"].as<std::string>();
	}

	if(!vm.count("outdir"))  {
		ret.out_dir = "";
	}
	else {
		ret.out_dir = vm["outdir"].as<std::string>();
	}

	if(!vm.count("outvid"))  {
		ret.out_vid = "";
	}
	else {
		ret.out_vid = vm["outvid"].as<std::string>();
	}

	if(!vm.count("showimg"))  {
		ret.showimg = false;
	}
	else {
		ret.showimg = vm["showimg"].as<bool>();
	}

	if(!vm.count("cam_estimate"))  {
		ret.cam_estimate = true;
	}
	else {
		ret.cam_estimate = vm["cam_estimate"].as<bool>();
	}

	if(!vm.count("interaction_on"))  {
		ret.interaction_on = false;
	}
	else {
		ret.interaction_on = vm["interaction_on"].as<bool>();
	}

	if(!vm.count("fps"))  { ret.fps = 15; }
	else {	ret.fps = vm["fps"].as<double>();	}

	if(!vm.count("iframe"))  { ret.iframe = 0; }
	else {	ret.iframe = vm["iframe"].as<int>();	}

	if(!vm.count("eframe"))  { ret.eframe = 100000; }
	else {	ret.eframe = vm["eframe"].as<int>();	}

	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	// observation parameters
	//////////////////////////////////////////////////////////////////
	if(!vm.count("min_height"))  { ret.min_height = 1.2; }
	else {	ret.min_height = vm["min_height"].as<double>();	}
	if(!vm.count("max_height"))  { ret.max_height = 2.2; }
	else {	ret.max_height = vm["max_height"].as<double>();	}
	if(!vm.count("feat_sigma_u"))  { ret.feat_sigma_u = 2; }
	else {	ret.feat_sigma_u = vm["feat_sigma_u"].as<double>();	}
	if(!vm.count("feat_sigma_v"))  { ret.feat_sigma_v = 2; }
	else {	ret.feat_sigma_v = vm["feat_sigma_v"].as<double>();	}
	if(!vm.count("det_scale"))  {	ret.det_scale = 1.0718;	}
	else {	ret.det_scale = vm["det_scale"].as<double>();	}
	if(!vm.count("det_weight"))  {	ret.det_weight = 1;	}
	else {	ret.det_weight = vm["det_weight"].as<double>();	}
	if(!vm.count("det_threshold"))  { ret.det_threshold = 0;	}
	else {	ret.det_threshold = vm["det_threshold"].as<double>(); }

	if(!vm.count("det_std_x"))  { ret.detection_std_x = 0.05;	}
	else {	ret.detection_std_x = vm["det_std_x"].as<double>(); }
	if(!vm.count("det_std_y"))  { ret.detection_std_y = 0.1;	}
	else {	ret.detection_std_y = vm["det_std_y"].as<double>(); }
	if(!vm.count("det_std_h"))  { ret.detection_std_h = 0.1;	}
	else {	ret.detection_std_h = vm["det_std_h"].as<double>(); }
	//////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////
	// sampling parameters
	//////////////////////////////////////////////////////////////////
	if(!vm.count("dbg_show"))  { ret.dbg_show = false; }
	else {	ret.dbg_show = vm["dbg_show"].as<bool>();	}
	if(!vm.count("num_samples"))  { ret.num_samples = 5000; }
	else {	ret.num_samples  = vm["num_samples"].as<int>();	}
	if(!vm.count("num_burnin"))  { ret.num_burnin = 1000; }
	else {	ret.num_burnin = vm["num_burnin"].as<int>();	}
	if(!vm.count("num_thinning"))  { ret.num_thinning = 100; }
	else {	ret.num_thinning = vm["num_thinning"].as<int>();	}
	if(!vm.count("min_samples"))  { ret.min_samples = 5; }
	else {	ret.min_samples = vm["min_samples"].as<int>();	}
	if(!vm.count("max_feats"))  { ret.max_feats = 40; }
	else {	ret.max_feats = vm["max_feats"].as<int>();	}
	//////////////////////////////////////////////////////////////////
#if 1
	//////////////////////////////////////////////////////////////////
	// initial camera parameters
	//////////////////////////////////////////////////////////////////
	if(!vm.count("init_cam_focal"))  { 
		std::cout << "init_cam_focal must be provided." << std::endl; 		
		exit(1);
	}
	else {	ret.init_cam_focal = vm["init_cam_focal"].as<double>();	}

	if(!vm.count("init_cam_x"))  { 
		std::cout << "init_cam_x must be provided." << std::endl; 		
		exit(1);
	}
	else {	ret.init_cam_x = vm["init_cam_x"].as<double>();	}
	if(!vm.count("init_cam_y"))  {
		std::cout << "init_cam_y must be provided." << std::endl; 		
		exit(1);
	}
	else {	ret.init_cam_y = vm["init_cam_y"].as<double>();	}
	if(!vm.count("init_cam_z"))  {
		std::cout << "init_cam_z must be provided." << std::endl; 		
		exit(1);
	}
	else {	ret.init_cam_z = vm["init_cam_z"].as<double>();	}
	if(!vm.count("init_cam_v"))  { 
		std::cout << "init_cam_v must be provided." << std::endl; 		
		exit(1);
	}
	else {	ret.init_cam_v = vm["init_cam_v"].as<double>();	}
	if(!vm.count("init_cam_yaw"))  {
		std::cout << "init_cam_yaw must be provided." << std::endl; 		
		exit(1);
	}
	else {	ret.init_cam_yaw = vm["init_cam_yaw"].as<double>();	}
	if(!vm.count("init_cam_horizon"))  { 
		std::cout << "init_cam_horizon must be provided." << std::endl; 		
		exit(1);
	}
	else {	ret.init_cam_horizon = vm["init_cam_horizon"].as<double>();	}
	if(!vm.count("init_cam_xcenter"))  {
		std::cout << "init_cam_xcenter must be provided." << std::endl; 		
		exit(1);
	}
	else {	ret.init_cam_xcenter = vm["init_cam_xcenter"].as<double>();	}
	//////////////////////////////////////////////////////////////////
#endif
	//////////////////////////////////////////////////////////////////
	// person motion parameters
	//////////////////////////////////////////////////////////////////
	if(!vm.count("motion_std_x"))  { ret.motion_std_x = 0.5; }
	else {	ret.motion_std_x = vm["motion_std_x"].as<double>();	}
	if(!vm.count("motion_std_y"))  { ret.motion_std_y = 0.1; }
	else {	ret.motion_std_y = vm["motion_std_y"].as<double>();	}
	if(!vm.count("motion_std_z"))  { ret.motion_std_z = 0.5; }
	else {	ret.motion_std_z = vm["motion_std_z"].as<double>();	}
	if(!vm.count("motion_std_vx"))  { ret.motion_std_vx = 3.0; }
	else {	ret.motion_std_vx = vm["motion_std_vx"].as<double>();	}
	if(!vm.count("motion_std_vz"))  { ret.motion_std_vz = 3.0; }
	else {	ret.motion_std_vz = vm["motion_std_vz"].as<double>();	}
	//////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////
	// interaction paramters
	//////////////////////////////////////////////////////////////////
	if(!vm.count("repulsion_const"))  { ret.repulsion_const = 2.0; }
	else {	ret.repulsion_const = vm["repulsion_const"].as<double>();	}
	if(!vm.count("group_const"))  { ret.group_const = 12.0; }
	else {	ret.group_const = vm["group_const"].as<double>();	}
	if(!vm.count("group_slope"))  { ret.group_slope = 3.0; }
	else {	ret.group_slope = vm["group_slope"].as<double>();	}
	if(!vm.count("group_threshold"))  { ret.group_threshold = 1.0; }
	else {	ret.group_threshold = vm["group_threshold"].as<double>();	}
	//////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////
	// person sample parameters
	//////////////////////////////////////////////////////////////////
	if(!vm.count("sample_std_x"))  { ret.sample_std_x = 0.2; }
	else {	ret.sample_std_x = vm["sample_std_x"].as<double>();	}
	if(!vm.count("sample_std_y"))  { ret.sample_std_y = 0.04; }
	else {	ret.sample_std_y = vm["sample_std_y"].as<double>();	}
	if(!vm.count("sample_std_z"))  { ret.sample_std_z = 0.2; }
	else {	ret.sample_std_z = vm["sample_std_z"].as<double>();	}
	if(!vm.count("sample_std_vx"))  { ret.sample_std_vx = 1.0; }
	else {	ret.sample_std_vx = vm["sample_std_vx"].as<double>();	}
	if(!vm.count("sample_std_vz"))  { ret.sample_std_vz = 1.0; }
	else {	ret.sample_std_vz = vm["sample_std_vz"].as<double>();	}
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	// camera motion parameters
	//////////////////////////////////////////////////////////////////
	if(!vm.count("cam_motion_std_x"))  { ret.cam_motion_std_x = 0.5; }
	else {	ret.cam_motion_std_x = vm["cam_motion_std_x"].as<double>();	}
	if(!vm.count("cam_motion_std_z"))  { ret.cam_motion_std_z = 0.5; }
	else {	ret.cam_motion_std_z = vm["cam_motion_std_z"].as<double>();	}
	if(!vm.count("cam_motion_std_v"))  { ret.cam_motion_std_v = 3.0; }
	else {	ret.cam_motion_std_v = vm["cam_motion_std_v"].as<double>();	}
	if(!vm.count("cam_motion_std_yaw"))  { ret.cam_motion_std_yaw = 0.1; }
	else {	ret.cam_motion_std_yaw = vm["cam_motion_std_yaw"].as<double>();	}
	if(!vm.count("cam_motion_std_horizon"))  { ret.cam_motion_std_horizon = 30; }
	else {	ret.cam_motion_std_horizon = vm["cam_motion_std_horizon"].as<double>();	}
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	// camera sample parameters
	//////////////////////////////////////////////////////////////////
	if(!vm.count("cam_sample_std_x"))  { ret.cam_sample_std_x = 0.2; }
	else {	ret.cam_sample_std_x = vm["cam_sample_std_x"].as<double>();	}
	if(!vm.count("cam_sample_std_z"))  { ret.cam_sample_std_z = 0.2; }
	else {	ret.cam_sample_std_z = vm["cam_sample_std_z"].as<double>();	}
	if(!vm.count("cam_sample_std_v"))  { ret.cam_sample_std_v = 1.0; }
	else {	ret.cam_sample_std_v = vm["cam_sample_std_v"].as<double>();	}
	if(!vm.count("cam_sample_std_yaw"))  { ret.cam_sample_std_yaw = 0.05; }
	else {	ret.cam_sample_std_yaw = vm["cam_sample_std_yaw"].as<double>();	}
	if(!vm.count("cam_sample_std_horizon"))  { ret.cam_sample_std_horizon = 10; }
	else {	ret.cam_sample_std_horizon = vm["cam_sample_std_horizon"].as<double>();	}
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	// feature sample parameters
	//////////////////////////////////////////////////////////////////
	if(!vm.count("feat_sample_std_x"))  { ret.feat_sample_std_x = 0.03; }
	else {	ret.feat_sample_std_x = vm["feat_sample_std_x"].as<double>();	}
	if(!vm.count("feat_sample_std_z"))  { ret.feat_sample_std_z = 0.03; }
	else {	ret.feat_sample_std_z = vm["feat_sample_std_z"].as<double>();	}
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	// sample parameters
	//////////////////////////////////////////////////////////////////
	if(!vm.count("prob_move_add"))  { ret.prob_move_add = 0.025; }
	else {	ret.prob_move_add = vm["prob_move_add"].as<double>();	}
	if(!vm.count("prob_move_delete"))  { ret.prob_move_delete = 0.025; }
	else {	ret.prob_move_delete = vm["prob_move_delete"].as<double>();	}
	if(!vm.count("prob_move_stay"))  { ret.prob_move_stay = 0.05; }
	else {	ret.prob_move_stay = vm["prob_move_stay"].as<double>();	}
	if(!vm.count("prob_move_leave"))  { ret.prob_move_leave = 0.05; }
	else {	ret.prob_move_leave = vm["prob_move_leave"].as<double>();	}
	if(!vm.count("prob_move_update"))  { ret.prob_move_update = 0.2; }
	else {	ret.prob_move_update = vm["prob_move_update"].as<double>();	}
	if(!vm.count("prob_move_feat_stay"))  { ret.prob_move_feat_stay = 0.075; }
	else {	ret.prob_move_feat_stay = vm["prob_move_feat_stay"].as<double>();	}
	if(!vm.count("prob_move_feat_leave"))  { ret.prob_move_feat_leave = 0.075; }
	else {	ret.prob_move_feat_leave = vm["prob_move_feat_leave"].as<double>();	}
	if(!vm.count("prob_move_feat_update"))  { ret.prob_move_feat_update = 0.1; }
	else {	ret.prob_move_feat_update = vm["prob_move_feat_update"].as<double>();	}
	if(!vm.count("prob_move_cam_update"))  { ret.prob_move_cam_update = 0.3; }
	else {	ret.prob_move_cam_update = vm["prob_move_cam_update"].as<double>();	}
	if(!vm.count("prob_move_interaction_flip"))  { ret.prob_move_interaction_flip = 0.1; }
	else {	ret.prob_move_interaction_flip = vm["prob_move_interaction_flip"].as<double>();	}
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	// existance parameters
	//////////////////////////////////////////////////////////////////
	if(!vm.count("prob_enter"))  { ret.prob_enter = 0.2; }
	else {	ret.prob_enter = vm["prob_enter"].as<double>();	}
	if(!vm.count("prob_stay"))  { ret.prob_stay = 0.8; }
	else {	ret.prob_stay = vm["prob_stay"].as<double>();	}
	if(!vm.count("prob_feat_enter"))  { ret.prob_feat_enter = 0.2; }
	else {	ret.prob_feat_enter = vm["prob_feat_enter"].as<double>();	}
	if(!vm.count("prob_feat_stay"))  { ret.prob_feat_stay = 0.9; }
	else {	ret.prob_feat_stay = vm["prob_feat_stay"].as<double>();	}
	//////////////////////////////////////////////////////////////////
	return ret;
}

ObservationManager *createManager()
{
	// create Observation Manager
	ObservationManager *mgr = new ObservationManager;
	ObservationNode *node;

	node = new DetectionReadinNode;
	mgr->insertObservationNode(node);
#if 0
	if(det_use_face_) {
		node = new FaceNode;
		mgr->insertObservationNode(node);
	}
#endif
	return mgr;
}

cv::Scalar get_target_color(int id) {
	cv::Scalar color = cv::Scalar(((id * 120) % 256), ((id * 60) % 256), ((id * 30) % 256));
	return color;
}

cv::Mat draw_features(ObservationManager *mgr, PosteriorDistPtr dist, cv::Mat &image_color, const std::vector<int> &remove_idx, bool bshow = true)
{
	cv::Mat image = image_color.clone();
	CamStatePtr mean_cam = dist->getMeanCamera();
	
	int fsize1, fsize2, fsize3;
	fsize1 = floor(image_color.cols / 400);
	fsize2 = floor(image_color.cols / 200);
	fsize3 = floor(image_color.cols / 50);

	for(int i = 0; i < dist->getNumFeats(); i++) {
		cv::Point2f pt2;
		FeatureDistPtr feat_dist = dist->getFeatureDist(i);
		std::vector<GFeatStatePtr> feats = feat_dist->getAllSamples();
		for(size_t j = 0; j < feats.size(); j++) {
			cv::Point2f pt = mean_cam->project(feats[j]);
			pt2.x = pt.x;
			pt2.y = pt.y;
			cv::circle(image, pt2, fsize1, cv::Scalar(10, 10, 10), CV_FILLED);
		}
	}

	std::vector<cv::Point2f> all_feats = mgr->getAllFeats();
	for(size_t i = 0; i < all_feats.size(); i++) {
		cv::Rect rt;
		rt.x = all_feats[i].x - fsize3/2;
		rt.y = all_feats[i].y - fsize3/2;
		rt.width = fsize3; rt.height = fsize3;

		cv::Scalar col = cv::Scalar(155, 0, 155);
		if(std::find(remove_idx.begin(), remove_idx.end(), i) != remove_idx.end()) {
			col = cv::Scalar(255, 255, 255);
		}
		cv::rectangle(image, rt.tl(), rt.br(), col, fsize2);
	}
	
	assert(all_feats.size() == dist->getNumFeats());
	
	if(bshow) {
#if 1
		show_image(image, "features", 600);
#else
		cv::imshow("features", image);
		cv::waitKey(30);
#endif
	}

	return image;
}

cv::Mat draw_features2(TargetManager &mgr, double ts, cv::Mat &image_color, const std::vector<int> &remove_idx, bool bshow = true)
{
	cv::Mat image = image_color.clone();
	image = mgr.drawFeatures(image, remove_idx, ts);

	if(bshow) {
#if 1
		show_image(image, "features", 600);
#else
		cv::imshow("features", image);
		cv::waitKey(30);
#endif
	}

	return image;
}

void draw_confidence_map(ObservationManager *mgr, double height, CamStatePtr mcam)
{
	cv::Mat cmap = mgr->getPeopleConfidenceMap(height, mcam, "all");
	cv::Mat cmap2(200, 200, CV_8U);
	for(int r = 0; r < 200; r++) {
		for(int c = 0; c < 200; c++) {
			float tempval = cmap.at<float>(r, c) ;

			tempval = std::min(tempval, 5.0f);
			tempval = std::max(tempval, -5.0f);

			cmap2.at<uchar>(r, c) = floor(tempval * 20 + 100);
		}
	}
#if 1
	show_image(cmap2, "conf", 600);
#else
	cv::imshow("conf", cmap2);
#endif
}

void draw_detections(ObservationManager *mgr, cv::Mat &image_color)
{
	// visualization
	cv::Mat image = image_color.clone();

	std::vector<cv::Rect> dets;
	int thickline = floor(image.cols / 300);

	mgr->quaryData("detection_readin_detection", &dets);
	for(size_t j = 0; j < dets.size(); j++) {
		cv::rectangle(image, dets[j].tl(), dets[j].br(), cv::Scalar(0, 0, 150), thickline);
	}
#if 1
	show_image(image, "detection", 600);
#else
	cv::imshow("detection", image);
	cv::waitKey(30);
#endif
}

cv::Mat draw_targets(TargetManager &manager, cv::Mat &image_color, double timestamp, bool bshow = true)
{
	cv::Mat image = manager.drawTargets(image_color, timestamp);
	if(bshow) { 
#if 1
		show_image(image, "targets", 600);
#else
		cv::imshow("targets", image);
#endif
	}
/*
	image = image_color.clone();
	std::vector<cv::Rect> rts = manager.getMSRects();
	std::vector<float> sims = manager.getMSSims();

	for(size_t i = 0; i < rts.size(); i++) {
		cv::rectangle(image, rts[i].tl(), rts[i].br(), cv::Scalar(255, 255, 255), 2);

		ostringstream text;
		text << std::setprecision(2) << sims[i];
		cv::putText(image, text.str(), rts[i].tl(), cv::FONT_HERSHEY_SIMPLEX, .4, cv::Scalar(0,0,0), 1);
	}

	cv::imshow("mean_shift", image);
	cv::waitKey(30);
*/
	return image;
}

cv::Mat draw_samples(PosteriorDistPtr dist, cv::Mat &image_color, bool bshow = true)
{
	cv::Mat image = image_color.clone();
	cv::Mat image_top = cv::Mat(400, 400, CV_8UC3, cv::Scalar(255, 255, 255));
	
	int nsamples = dist->getNumSamples();
	int ntargets;

	MCMCSamplePtr sample;
	CamStatePtr cam;
	PeopleStatePtr ped;
	cv::Rect rt;
	
	int thinline = floor(image.cols / 600);
	int thickline = floor(image.cols / 300);
	int boxsize = floor(image.cols / 20);

	cv::Mat camera_view(image, cv::Rect(boxsize, boxsize, 2 * boxsize, 2 * boxsize));

	camera_view = cv::Scalar(255, 255, 255);
	cv::rectangle(camera_view, cv::Point(0,0), cv::Point(2 * boxsize, 2 * boxsize), cv::Scalar(0,0,0), thickline);

	for( int i = 0; i < nsamples; i++ ) {
		sample = dist->getSample(i);

		ntargets = sample->getNumTargets();
		cam = sample->getCamState();
		cv::line(image, cv::Point(0, cam->getHorizon()), cv::Point(image.cols, cam->getHorizon()), cv::Scalar(255, 0, 255), thinline);
		// draw view point
		double angle = cam->getYaw();
		angle = -M_PI / 2 - angle;
		double x = cos(angle), z = sin(angle);
		// box
		cv::line(camera_view, cv::Point(boxsize, boxsize), cv::Point(boxsize + boxsize * x, boxsize + boxsize * z), cv::Scalar(0,0,0), thinline);

		// samples on image plane
		for( int j = 0; j < ntargets; j++ ) {
			if(sample->getExistance(j)) {
				ped = sample->getState(j);
				rt = cam->project(ped);
				cv::rectangle(image, rt.tl(), rt.br(), cv::Scalar(150, 100, 100), 1);

				for(size_t k = 0; k < j; k++) {
					if(sample->getExistance(k) && sample->getInteractionMode(k, j)) {
						cv::Rect rt2 = cam->project(sample->getState(k));

						cv::line(image, cv::Point(rt.x + rt.width / 2, rt.y + rt.height), cv::Point(rt2.x + rt2.width / 2, rt2.y + rt2.height), cv::Scalar(100, 0, 100), 1);
					}
				}
			}
		}

		// samples on top view
		for( int j = 0; j < ntargets; j++ ) {
			if(sample->getExistance(j)) {
				ped = sample->getState(j);
				cv::Point pt;

				// 1 meter == 10 pixel
				pt.y = ( ped->getX() - cam->getX() ) * 10.0 + 200;
				pt.x = ( ped->getZ() - cam->getZ() ) * 10.0 + 200;

				cv::circle(image_top, pt, 1, get_target_color(j), 3);
			}
		}
	}

	if(bshow) {
#if 1
		show_image(image, "samples", 600);
#else
		cv::imshow("samples", image);
		cv::waitKey(30);
#endif
	}

	if(bshow) {
#if 1
		show_image(image_top, "top view", 600);
#else
		cv::imshow("top view", image_top);
		cv::waitKey(30);
#endif
	}

	return image;
}

/////////////////////////////////////////////////////////////////////
int main (int ac, char** av)
{
	param_set params = parse_arguments(ac, av);
	
	///////////////////////////////////////////////////////////////
	// read input list
	///////////////////////////////////////////////////////////////
	std::vector<std::string> im_files;
	std::vector<std::string> conf_files;
	std::vector<std::string> vp_files;

	im_files = read_file_list(params.im_list_file);
	conf_files = read_file_list(params.conf_list_file);
	if(im_files.size() == 0 || conf_files.size() == 0) {
		std::cout << "no valid imfile/conffile" << std::endl;
		return -1;
	}

	if(params.vp_list_file != "") {
		vp_files = read_file_list(params.vp_list_file);
	}

	//////////////////////////////////////////////////////////////
	cv::VideoWriter video_target;
	cv::VideoWriter video_samples;
	cv::VideoWriter video_feature;
	cv::VideoWriter video_camera;

	double fps = params.fps;
	if(params.out_vid != "") {
		std::string fname = params.out_vid + "_target.avi";
		if(!video_target.open(fname, CV_FOURCC('X','V','I','D'), fps, cv::Size(640 , 480), true))
		my_assert(video_target.isOpened());

		fname = params.out_vid + "_samples.avi";
		if(!video_samples.open(fname, CV_FOURCC('X','V','I','D'), fps, cv::Size(640 , 480), true))
		my_assert(video_samples.isOpened());

		fname = params.out_vid + "_feature.avi";
		if(!video_feature.open(fname, CV_FOURCC('X','V','I','D'), fps, cv::Size(640 , 480), true))
		my_assert(video_feature.isOpened());
	}

	///////////////////////////////////////////////////////////////
	TargetManager		target_manager;
	ObservationManager 	*mgr = createManager();
	RJMCMCTracker 		tracker;
	FeatTracker			feat_tracker;

	double timesec;
	// set parameters
	mgr->setParameters("min_height", boost::lexical_cast<std::string>(params.min_height));
	mgr->setParameters("max_height", boost::lexical_cast<std::string>(params.max_height));
	mgr->setParameters("feat_sigma_u", boost::lexical_cast<std::string>(params.feat_sigma_u));
	mgr->setParameters("feat_sigma_v", boost::lexical_cast<std::string>(params.feat_sigma_u));
	mgr->setParameters("detection_readin_weight", boost::lexical_cast<std::string>(params.det_weight));
	mgr->setParameters("detection_readin_threshold", boost::lexical_cast<std::string>(params.det_threshold));
	mgr->setParameters("detection_readin_det_scale", boost::lexical_cast<std::string>(params.det_scale));
	mgr->setParameters("detection_std_x", boost::lexical_cast<std::string>(params.detection_std_x));
	mgr->setParameters("detection_std_y", boost::lexical_cast<std::string>(params.detection_std_y));
	mgr->setParameters("detection_std_h", boost::lexical_cast<std::string>(params.detection_std_h));
	//
	tracker.setParameters("ShowDebugMsg", boost::lexical_cast<std::string>(params.dbg_show));
	tracker.setParameters("MCMCNumSamples", boost::lexical_cast<std::string>(params.num_samples));
	tracker.setParameters("MCMCBurnin", boost::lexical_cast<std::string>(params.num_burnin));
	tracker.setParameters("MCMCThinning", boost::lexical_cast<std::string>(params.num_thinning));
	tracker.setParameters("MaximumGFeats", boost::lexical_cast<std::string>(params.max_feats));

	tracker.setParameters("InteractionOn", boost::lexical_cast<std::string>(params.interaction_on));
	
	tracker.setParameters("RepulsionConstant", boost::lexical_cast<std::string>(params.repulsion_const));
	tracker.setParameters("GroupConstant", boost::lexical_cast<std::string>(params.group_const));
	tracker.setParameters("GroupSlopeSigmoid", boost::lexical_cast<std::string>(params.group_slope));
	tracker.setParameters("GroupThresholdSigmoid", boost::lexical_cast<std::string>(params.group_threshold));

	tracker.setParameters("EstimateCamera", boost::lexical_cast<std::string>(params.cam_estimate));

	tracker.setParameters("ProbMoveAdd", boost::lexical_cast<std::string>(params.prob_move_add));
	tracker.setParameters("ProbMoveDelete", boost::lexical_cast<std::string>(params.prob_move_delete));
	tracker.setParameters("ProbMoveStay", boost::lexical_cast<std::string>(params.prob_move_stay));
	tracker.setParameters("ProbMoveLeave", boost::lexical_cast<std::string>(params.prob_move_leave));
	tracker.setParameters("ProbMoveUpdate", boost::lexical_cast<std::string>(params.prob_move_update));
	tracker.setParameters("ProbMoveFeatStay", boost::lexical_cast<std::string>(params.prob_move_feat_stay));
	tracker.setParameters("ProbMoveFeatLeave", boost::lexical_cast<std::string>(params.prob_move_feat_leave));
	tracker.setParameters("ProbMoveFeatUpdate", boost::lexical_cast<std::string>(params.prob_move_feat_update));
	tracker.setParameters("ProbMoveCamUpdate", boost::lexical_cast<std::string>(params.prob_move_cam_update));
	tracker.setParameters("ProbMoveInteractionFlip", boost::lexical_cast<std::string>(params.prob_move_interaction_flip));

	tracker.setParameters("ProbEnter", boost::lexical_cast<std::string>(params.prob_enter));
	tracker.setParameters("ProbStay", boost::lexical_cast<std::string>(params.prob_stay));
	tracker.setParameters("ProbFeatEnter", boost::lexical_cast<std::string>(params.prob_feat_enter));
	tracker.setParameters("ProbFeatStay", boost::lexical_cast<std::string>(params.prob_feat_stay));

	tracker.setParameters("MotionSigmaX", boost::lexical_cast<std::string>(params.motion_std_x));
	tracker.setParameters("MotionSigmaY", boost::lexical_cast<std::string>(params.motion_std_y));
	tracker.setParameters("MotionSigmaZ", boost::lexical_cast<std::string>(params.motion_std_z));
	tracker.setParameters("MotionSigmaVX", boost::lexical_cast<std::string>(params.motion_std_vx));
	tracker.setParameters("MotionSigmaVZ", boost::lexical_cast<std::string>(params.motion_std_vz));
	tracker.setParameters("PerturbSigmaX", boost::lexical_cast<std::string>(params.sample_std_x));
	tracker.setParameters("PerturbSigmaY", boost::lexical_cast<std::string>(params.sample_std_y));
	tracker.setParameters("PerturbSigmaZ", boost::lexical_cast<std::string>(params.sample_std_z));
	tracker.setParameters("PerturbSigmaVX", boost::lexical_cast<std::string>(params.sample_std_vx));
	tracker.setParameters("PerturbSigmaVZ", boost::lexical_cast<std::string>(params.sample_std_vz));

	tracker.setParameters("CameraMotionSigmaX", boost::lexical_cast<std::string>(params.cam_motion_std_x));
	tracker.setParameters("CameraMotionSigmaZ", boost::lexical_cast<std::string>(params.cam_motion_std_z));
	tracker.setParameters("CameraMotionSigmaV", boost::lexical_cast<std::string>(params.cam_motion_std_v));
	tracker.setParameters("CameraMotionSigmaYaw", boost::lexical_cast<std::string>(params.cam_motion_std_yaw));
	tracker.setParameters("CameraMotionSigmaHorizon", boost::lexical_cast<std::string>(params.cam_motion_std_horizon));
	tracker.setParameters("PerturbCamSigmaX", boost::lexical_cast<std::string>(params.cam_sample_std_x));
	tracker.setParameters("PerturbCamSigmaZ", boost::lexical_cast<std::string>(params.cam_sample_std_z));
	tracker.setParameters("PerturbCamSigmaV", boost::lexical_cast<std::string>(params.cam_sample_std_v));
	tracker.setParameters("PerturbCamSigmaYaw", boost::lexical_cast<std::string>(params.cam_sample_std_yaw));
	tracker.setParameters("PerturbCamSigmaHorizon", boost::lexical_cast<std::string>(params.cam_sample_std_horizon));

	tracker.setParameters("PerturbFeatSigmaX", boost::lexical_cast<std::string>(params.feat_sample_std_x));
	tracker.setParameters("PerturbFeatSigmaZ", boost::lexical_cast<std::string>(params.feat_sample_std_z));

	tracker.setParameters("DetectionSigmaX", boost::lexical_cast<std::string>(params.detection_std_x));
	tracker.setParameters("DetectionSigmaY", boost::lexical_cast<std::string>(params.detection_std_y));
	tracker.setParameters("DetectionSigmaH", boost::lexical_cast<std::string>(params.detection_std_h));
	
	// initialize the first frame camera
	CamStatePtr init_cam = boost::make_shared<CamState>();
	init_cam->set(params.init_cam_x,	params.init_cam_y, params.init_cam_z,
				params.init_cam_v,	params.init_cam_yaw, params.init_cam_horizon,
				params.init_cam_focal, params.init_cam_xcenter,	0.0);
	tracker.setInitialCamera(init_cam);
	// feat_tracker.showImage(true);
	mgr->setData((void*)&feat_tracker, "feat_tracker");
	for(size_t i = max(0, params.iframe); i < min((int)im_files.size(), params.eframe); i++) {
		timesec = (double)i / fps;
		// process one frame!!!
		std::string fname = params.root_dir + im_files[i];
		std::string cname = params.root_dir + conf_files[i];

		std::string vpname = "";
		if(vp_files.size() > 0) {
			vpname = params.root_dir + vp_files[i];
		}
		// get filenames 
		std::cout << "process " << fname << std::endl;
#if 1
		cv::Mat image_color = cv::imread(fname);
		cv::Mat image_mono; 
		cvtColor(image_color, image_mono, CV_BGR2GRAY);
#else
		cv::Mat image_color_big = cv::imread(fname);
		cv::Mat image_color;
		cv::resize(image_color_big, image_color, cv::Size(500, 500));
		cv::Mat image_mono; 
		cvtColor(image_color, image_mono, CV_BGR2GRAY);
#endif
		/////////////////////////////////////////////////////////////////////////////////////////////////
		// set data
		/////////////////////////////////////////////////////////////////////////////////////////////////
		mgr->setData((void*)&cname, "detection_readin_conf_file");
		mgr->setData((void*)&vpname, "vp_estimate_file");
		mgr->setData((void*)&image_color, "image_color");
		mgr->setData((void*)&image_mono, "image_mono");
		mgr->setData((void*)&timesec, "time_sec");
		/////////////////////////////////////////////////////////////////////////////////////////////////

		/////////////////////////////////////////////////////////////////////////////////////////////////
		// process detector
		/////////////////////////////////////////////////////////////////////////////////////////////////
		mgr->preprocess();
#if 0
		CamStatePtr temptemp = boost::make_shared<CamState>();
		temptemp->set(params.init_cam_x,	params.init_cam_y, params.init_cam_z,
					params.init_cam_v,	params.init_cam_yaw, params.init_cam_horizon,
					params.init_cam_focal, params.init_cam_xcenter,	0.0);
		for(int iii = 1; iii < 2000; iii++) {
			temptemp->setHorizon(iii);
			std::cout << mgr->getCameraConfidence(temptemp)	<< setprecision(4) << " ";
		}
		std::cout << std::endl;
		return 0;
#endif
		// run/set meanshift data
		target_manager.runMeanShift(image_color);
		tracker.setMeanShiftData(target_manager.getMSRects(), target_manager.getMSSims());
		// generate proposals
		std::vector<cv::Rect> dets = mgr->getDetections();
		// find matches between new detections and existing targets
		std::vector<cv::Rect> proposal_rts;
		target_manager.getProposals(dets, image_color, proposal_rts); // using overlap between bbs
		tracker.setData(mgr, proposal_rts);
		/////////////////////////////////////////////////////////////////////////////////////////////////
		if(params.showimg) draw_detections(mgr, image_color);
		/////////////////////////////////////////////////////////////////////////////////////////////////
		// run tracking
		/////////////////////////////////////////////////////////////////////////////////////////////////
		tracker.runMCMCSampling();
		/////////////////////////////////////////////////////////////////////////////////////////////////

		/////////////////////////////////////////////////////////////////////////////////////////////////
		// arrange data and post process
		/////////////////////////////////////////////////////////////////////////////////////////////////
		PosteriorDistPtr posterior_dist = tracker.getPosterior();
		std::vector<TargetDistPtr> tracks = posterior_dist->getTargetDists();
		
		// process targets
		std::vector<cv::Rect> target_rts;
		CamStatePtr mean_cam = posterior_dist->getMeanCamera();
		for(size_t j = 0; j < tracks.size(); j++) {
			target_rts.push_back(mean_cam->project(tracks[j]->getMean()));
		}
		target_manager.updateTargets(tracks, target_rts, image_color);
		target_manager.updateMeanShiftTrackers(image_color);

		// filter targets ...
		std::vector<int> remove_idx = target_manager.getTerminatingTargets(0.1f, cv::Size(image_color.cols, image_color.rows));
		tracker.filterTargets(remove_idx);

		// process features ...
		std::vector<int> feat_idx = tracker.getFeatIdx();
		std::vector<FeatureDistPtr> feat_dists = posterior_dist->getFeatureDists();
		std::vector<cv::Point2f> obs = mgr->getAllFeats();
		std::vector<cv::Point2f> pts;
		
		assert(feat_idx.size() == feat_dists.size());
		assert(obs.size() == feat_dists.size());

		for(size_t j = 0; j < feat_dists.size(); j++) {
			pts.push_back(mean_cam->project(feat_dists[j]->getMean()));
		}
		target_manager.updateFeatures(feat_idx, feat_dists, pts, obs);

		// filter features ...
		remove_idx.clear();
		for(int j = 0; j < posterior_dist->getNumFeats(); j++) {
			GFeatStatePtr feat = posterior_dist->getMeanFeature(j);
			// feat->print();
			if(posterior_dist->getMeanFeatValidity(j) < 0.2 || 
				pts[j].y < mean_cam->getHorizon() + 20) { // safeguard
				remove_idx.push_back(j);
				// std::cout << "============> remove " << j << std::endl;
			}
		}
		cv::Mat frame;
		frame = draw_features(mgr, posterior_dist, image_color, remove_idx, params.showimg);
		// frame = draw_features2(target_manager, timesec, image_color, remove_idx, params.showimg);
		if(video_feature.isOpened()) {
			video_feature << frame;
		}

		tracker.filterFeatures(remove_idx);
		target_manager.updateCamera(mean_cam);
		/////////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////////////////////////
		// draw functions
		/////////////////////////////////////////////////////////////////////////////////////////////////
		if(params.showimg) draw_confidence_map(mgr, 1.7, mean_cam);
		frame = draw_targets(target_manager, image_color, timesec, params.showimg);
		if(video_target.isOpened()) {
			video_target << frame;
		}

		frame = draw_samples(posterior_dist, image_color, params.showimg);
		if(video_samples.isOpened()) {
			video_samples << frame;
		}
#if 1
#else
		cv::waitKey();
#endif
		/////////////////////////////////////////////////////////////////////////////////////////////////
	}
	assert(im_files.size() == conf_files.size());

	target_manager.saveAll(params.out_dir);

	delete mgr;
}
