#ifndef RJMCMC_TRACKER_H_
#define RJMCMC_TRACKER_H_

#include <sys/time.h>
#include <opencv/cv.h>
#include <common/ped_state.h>
#include <common/mcmc_sample.h>
#include <prior/PriorDist.h>
#include <proposal/rjmcmc_proposal.h>
#include <observation/ObservationManager.h>

#ifndef CAM_EST
error
#endif

namespace people {
	class MeanShiftWrapper
	{
	public:
		MeanShiftWrapper():sigma_x_(0.1),sigma_y_(0.15),sigma_h_(0.15) {};
		virtual ~MeanShiftWrapper() {};

		void setData(std::vector<cv::Rect> &rts, std::vector<float> &sims) {
			ms_rts_ = rts;
			ms_sims_ = sims;
		}

		void computeNewSampleMSLkhood(const SampleInfo &info, MCMCSamplePtr sample)
		{
			if(info.idx_ >= ms_rts_.size()) {
				return; // new target nothing to be done
			}
			my_assert(info.type_ != MoveAdd);
			my_assert(info.type_ != MoveDelete);

			if((info.type_ == MoveStay) || (info.type_ == MoveUpdate)) {
				PeopleStatePtr state = info.state_;
				CamStatePtr cam = sample->getCamState();
				
				new_people_cache_[info.idx_] = computeMotionLkhood(state, cam, info.idx_);
			}
			else if(info.type_ == MoveLeave) {
				new_people_cache_[info.idx_] = 0.0f;
			}
			else if(info.type_ == MoveCamUpdate) {
				double lkhood;
				CamStatePtr cam = info.cam_state_;
				for(size_t i = 0 ; i < ms_rts_.size(); i++) {
					if(sample->getExistance(i)) {
						PeopleStatePtr state = sample->getState(i);
						lkhood = computeMotionLkhood(state, cam, i);
					}
					else {
						lkhood = 0.0;
					}
					new_people_cache_[i] = lkhood;
				}
			}
		}

		void initCache(MCMCSamplePtr sample)  
		{
			double lkhood;
			people_cache_.clear();
			for(int i = 0; i < (int)ms_rts_.size(); i++) {
				if(sample->getExistance(i)) {
					PeopleStatePtr state = sample->getState(i);
					CamStatePtr cam = sample->getCamState();

					lkhood = computeMotionLkhood(state, cam, i);
				}
				else {
					lkhood = 0.0;
				}
				people_cache_.push_back(lkhood);
			}
			new_people_cache_ = people_cache_;
		}

		double computeLogMSLkhood(const SampleInfo &info) {
			double ret = 0.0;
			switch(info.type_) {
			case MoveUpdate:
				if(info.idx_ < people_cache_.size()) {
					ret = (double)(new_people_cache_[info.idx_] - people_cache_[info.idx_]);
				}
				break;
			case MoveCamUpdate:
				for(size_t i = 0; i < new_people_cache_.size(); i++) {
					ret += (double)(new_people_cache_[i] - people_cache_[i]);
				}
				my_assert(!isnan(ret));
				break;
			case MoveStay:
			case MoveLeave:
			case MoveAdd:
			case MoveDelete:
			case MoveInteractionFlip:
			case MoveFeatAdd:
			case MoveFeatDelete:
			case MoveFeatStay:
			case MoveFeatLeave:
			case MoveFeatUpdate:
				break;
			default:
				assert(0);
			}
			return ret;
		}

		void updateCache(const SampleInfo &info, bool accepted) {
			if(!accepted) return;

			switch(info.type_) {
			case MoveStay:
			case MoveLeave:
			case MoveUpdate:
				if(info.idx_ < people_cache_.size()) {
					people_cache_[info.idx_] = new_people_cache_[info.idx_];
				}
				break;
			case MoveCamUpdate:
				people_cache_ = new_people_cache_;
				break;
			case MoveAdd:
			case MoveDelete:
			case MoveInteractionFlip:
			case MoveFeatAdd:
			case MoveFeatDelete:
			case MoveFeatStay:
			case MoveFeatLeave:
			case MoveFeatUpdate:
				break;
			default:
				assert(0);
			}
		};
	protected:
		double computeMotionLkhood(PeopleStatePtr state, CamStatePtr cam, int idx)
		{
			double ret = 0.0;
			// not reliable
			if(ms_sims_[idx] < 0.65) return ret;

			cv::Rect proj = cam->project(state);
			// look at both rectangle and sim value
			double x1, y1, h1;
			double x2, y2, h2;
			double sx, sy, sh;
			
			x1 = proj.x + proj.width / 2;
			h1 = proj.height;
			y1 = proj.y + h1 / 2;
			
			x2 = ms_rts_[idx].x + ms_rts_[idx].width / 2;
			h2 = ms_rts_[idx].height;
			y2 = ms_rts_[idx].y + h2 / 2;
			
			sx = h2 * sigma_x_;
			sy = h2 * sigma_y_;
			sh = h2 * sigma_h_;

			ret = - pow((x1 - x2) / sx, 2);
			ret += - pow((y1 - y2) / sy, 2);
			ret += - pow((h1 - h2) / sh, 2);
			ret *= 5;
			// ret /= 3; // maximum + 2
#if 0
			std::cout << "samp x:" << proj.x << "y:" << proj.y << "w:" << proj.width << "h:" << proj.height << std::endl;
			std::cout << "ms x:" << ms_rts_[idx].x << "y:" << ms_rts_[idx].y << "w:" << ms_rts_[idx].width << "h:" << ms_rts_[idx].height << std::endl;
			std::cout << "ret : " << ret << std::endl;
			assert(0);
#endif
			// ret = std::max(-1.0, ret);
			// ret = std::min(2.0, ret);

			// ret = 0.0;
			return ret;
		};

	protected:
		std::vector<double> people_cache_;
		std::vector<double> new_people_cache_;

		std::vector<cv::Rect> ms_rts_;
		std::vector<float> ms_sims_;
		// relative to the height
		double sigma_x_;
		double sigma_y_;
		double sigma_h_;
	};

	class ObservationWrapper
	{
	public:
		ObservationWrapper():obs_mgr_(NULL){};
		virtual ~ObservationWrapper(){};
		void setObsMgr(ObservationManager *mgr) {
			obs_mgr_ = mgr;
		}

		void computeNewSampleObservationLkhood(const SampleInfo &info, MCMCSamplePtr sample)
		{
			assert(obs_mgr_ != NULL);
			if((info.type_ == MoveAdd) || (info.type_ == MoveStay) || (info.type_ == MoveUpdate)) {
				new_people_cache_[info.idx_] = obs_mgr_->getPeopleConfidence(info.state_, sample->getCamState());
			}
			else if((info.type_ == MoveDelete) || (info.type_ == MoveLeave)) {
				new_people_cache_[info.idx_] = 0.0f;
			}
			// non pedestrian related observations
			else if (info.type_ == MoveInteractionFlip) {
				// nothing to be done
			}
			// feature related 
			else if((info.type_ == MoveFeatAdd) || (info.type_ == MoveFeatStay) || (info.type_ == MoveFeatUpdate)){
				new_feat_cache_[info.idx_] = obs_mgr_->getGFeatConfidence(info.feat_state_, info.idx_, sample->getCamState());
			}
			else if((info.type_ == MoveFeatDelete) || (info.type_ == MoveFeatLeave)){
				new_feat_cache_[info.idx_] = 0.0;
			}
			// camera related
			else if (info.type_ == MoveCamUpdate) {
				for(int i = 0; i < (int)sample->getNumTargets(); i++) {
					if(sample->getExistance(i)) {
						PeopleStatePtr state = sample->getState(i);
						new_people_cache_[i] = obs_mgr_->getPeopleConfidence(state, info.cam_state_);
					}
					else {
						new_people_cache_[i] = 0.0f;
						my_assert(people_cache_[i] == 0.0f);
					}
				}

				for(int i = 0; i < (int)sample->getNumGFeats(); i++) {
					if(sample->getFeatValidity(i)) {
						GFeatStatePtr state = sample->getFeatState(i);
						new_feat_cache_[i] = obs_mgr_->getGFeatConfidence(state, i, info.cam_state_);
					}
					else {
						new_feat_cache_[i] = 0.0f;
						my_assert(feat_cache_[i] == 0.0f);
					}
				}
			}
			else {
				// none defined
				assert(0);
			}
		}

		void initCache(MCMCSamplePtr sample)  
		{
			assert(obs_mgr_ != NULL);

			people_cache_.clear();
			for(int i = 0; i < (int)sample->getNumTargets(); i++) {
				if(sample->getExistance(i)) {
					PeopleStatePtr state = sample->getState(i);

					double lkhood = obs_mgr_->getPeopleConfidence(state, sample->getCamState());
					people_cache_.push_back(lkhood);
				}
				else {
					people_cache_.push_back(0.0f);
				}
			}
			
			feat_cache_.clear();
			for(int i = 0; i < (int)sample->getNumGFeats(); i++) {
				if(sample->getFeatValidity(i)) {
					GFeatStatePtr state = sample->getFeatState(i);

					double lkhood = obs_mgr_->getGFeatConfidence(state, i, sample->getCamState());
					feat_cache_.push_back(lkhood);
				}
				else {
					feat_cache_.push_back(0.0f);
				}
			}

			new_people_cache_ = people_cache_;
			new_feat_cache_ = feat_cache_;
		}

		double computeLogObservationLkhood(const SampleInfo &info, MCMCSamplePtr sample) {
			double ret = 0.0;
			switch(info.type_) {
			case MoveAdd:
			case MoveDelete:
			case MoveStay:
			case MoveLeave:
				// ret = soft_max((double)(new_people_cache_[info.idx_] - people_cache_[info.idx_]), 2.0);
				ret = (double)(new_people_cache_[info.idx_] - people_cache_[info.idx_]);
				break;
			case MoveUpdate:
				// 1000 -> 5times, 
				// 1001 -> 3times + 1/2 detection box
				ret = 5.0 * (double)(new_people_cache_[info.idx_] - people_cache_[info.idx_]);
				// ret = (double)(new_people_cache_[info.idx_] - people_cache_[info.idx_]);
				break;
			case MoveInteractionFlip:
				break;
			case MoveFeatAdd:
			case MoveFeatDelete:
			case MoveFeatStay:
			case MoveFeatLeave:
			case MoveFeatUpdate:
				ret = (double)(new_feat_cache_[info.idx_] - feat_cache_[info.idx_]);
				break;
			case MoveCamUpdate:
				//std::cout << "obs details : people dets : ";
				for(size_t i = 0; i < new_people_cache_.size(); i++) {
					ret += 5.0 * (double)(new_people_cache_[i] - people_cache_[i]);
					// ret += (double)(new_people_cache_[i] - people_cache_[i]);
				}
				//std::cout << ret ;
				for(size_t i = 0; i < new_feat_cache_.size(); i++) {
					ret += (double)(new_feat_cache_[i] - feat_cache_[i]);
				}
				//std::cout << " feats : " << ret ;
#if 0 // def HORIZON_EST
				// horizon observation using detections!!!
				{
					std::vector<int> votes;
					std::vector<double> std;
					CamStatePtr cam = sample->getCamState();
					obs_mgr_->getHorizonVotes(votes, std, cam->getY());
					// std::cout << "confidence : " << ret << std::endl;
					for(size_t i = 0; i < votes.size(); i++) {
						double new_dist = votes[i] - info.cam_state_->getHorizon();
						double old_dist = votes[i] - cam->getHorizon();
						ret += - min(pow(new_dist / std[i], 2), 9.0);
						ret -= - min(pow(old_dist / std[i], 2), 9.0);
						//std::cout << "vote " << votes[i] << " " ;
						//print_rect(dets[i]);
						//std::cout << "new_dist : " << new_dist << " " ;
						//info.cam_state_->print();
						//std::cout << "old_dist : " << old_dist << " " ;
						//cam->print();
					}
					//std::cout << "confidence : " << ret << std::endl;
					//cv::waitKey();
					//std::cout << " horizon : " << ret ;
				}
#endif
#if 1
				ret += obs_mgr_->getCameraConfidence(info.cam_state_) - obs_mgr_->getCameraConfidence(sample->getCamState());
#endif
				my_assert(!isnan(ret));
				break;
			default:
				assert(0);
			}

			return ret;
		}

		void updateCache(const SampleInfo &info, bool accepted) {
			if(!accepted) return;

			switch(info.type_) {
			case MoveAdd:
			case MoveDelete:
			case MoveStay:
			case MoveLeave:
			case MoveUpdate:
				people_cache_[info.idx_] = new_people_cache_[info.idx_];
				break;
			case MoveInteractionFlip:
				break;
			case MoveFeatAdd:
			case MoveFeatDelete:
			case MoveFeatStay:
			case MoveFeatLeave:
			case MoveFeatUpdate:
				feat_cache_[info.idx_] = new_feat_cache_[info.idx_];
				break;
			case MoveCamUpdate:
				people_cache_ = new_people_cache_;
				feat_cache_ = new_feat_cache_;
				break;
			default:
				assert(0);
			}
		}

		double computeLogObservationLkhood_nocache(MCMCSamplePtr sample, const SampleInfo &info) {
			double ret = 0;
			assert(0);
			return ret; 
		}

		double getTimeSec() { return obs_mgr_->getTimeSec(); };
		ObservationManager *getManager() { return obs_mgr_; };
	protected:
		ObservationManager *obs_mgr_;
		std::vector<double> people_cache_;
		std::vector<double> feat_cache_;
		std::vector<double> new_people_cache_;
		std::vector<double> new_feat_cache_;
	};
	//

	class RJMCMCTracker
	{
	public:
		RJMCMCTracker():rng_((double)time(NULL)),prev_dist_(boost::make_shared<PosteriorDist>(PosteriorDist()))
		{
			setDefaultParameters();
		}

		~RJMCMCTracker(){};
		
		// run one frame tracking
		void	runMCMCSampling();

		void setMeanShiftData(const std::vector<cv::Rect> &rts, const std::vector<float> &sims)
		{
			ms_rts_ = rts;
			ms_sims_ = sims;
		}

		// set detection and observation lkhood module
		void 	setData(ObservationManager *mgr, const std::vector<cv::Rect> &proposals) {
			setProposals(proposals); 
			setObservationManager(mgr);
		}
		void 	setProposals(const std::vector<cv::Rect> &proposals){ proposals_ = proposals; };
		void 	setObservationManager(ObservationManager *data) { obs_wrapper_.setObsMgr(data); };
		
		// initial camera
		void 	setInitialCamera(CamStatePtr cam) { init_cam_ = cam->clone(); }

		CamStatePtr initializeCamera();
		// set parameters
		void 	setParameters(const std::string &name, const std::string &value);
		void 	filterTargets(std::vector<int> &remove_idx);
		void 	filterFeatures(std::vector<int> &remove_idx);

		std::vector<int> getFeatIdx() {return feat_idx_;};
		inline PosteriorDistPtr getPosterior() {return prev_dist_;};
	protected:
		// remove the features that are not useful...
		void  prefilter_features(double timestamp);
		void 	normalizeMoveProb();
		void  setDefaultParameters();
		// double			getLogEnterProb(PedState *state) {return -1.0; /* log(0.1); */}; // temporary
	protected:
		// double 			computeLogAcceptanceRatio(MCMCSample *pOldSample, MCMCSample *pNewSample, MCMCCache &cache, SampleInfo &info);
		MCMCSamplePtr getInitSample(double timesec);
	protected:
		// random number generator..
		cv::RNG         			rng_;
		/////////////////////////////////////
		// previous distribution
		PosteriorDistPtr			prev_dist_;
		/////////////////////////////////////
		// mapping between features in 
		// feat tracker and samples
		std::vector<int>			feat_idx_;
		/////////////////////////////////////
		// input data...
		ObservationWrapper			obs_wrapper_;
		/////////////////////////////////////

		/////////////////////////////////////
		/// proposals given by external algorithm, eg. leg detector, depth info..
		std::vector<cv::Rect>			proposals_;
		/////////////////////////////////////

		/////////////////////////////////////
		// mean-shift data
		std::vector<cv::Rect>	ms_rts_;
		std::vector<float>		ms_sims_;
		/////////////////////////////////////

		/////////////////////////////////////
		// initial camera
		CamStatePtr init_cam_;
		/////////////////////////////////////

		/***********************************
		 ** parameters
		 **********************************/
		int			num_samples_;
		int			burnin_;
		int			thinning_;
		
		///////////////////////////////////////////
		// detection of people
		double 	detection_sigma_x_;
		double 	detection_sigma_y_;
		double 	detection_sigma_h_;
		///////////////////////////////////////////

		///////////////////////////////////////////
		// motion/sampling of people
		double 	motion_sigma_x_;
		double 	motion_sigma_y_;
		double 	motion_sigma_z_;
		double  perturb_x_;
		double  perturb_y_;
		double  perturb_z_;
		double 	motion_sigma_vx_;
		double 	motion_sigma_vz_;
		double  perturb_vx_;
		double  perturb_vz_;
		///////////////////////////////////////////

		///////////////////////////////////////////
		// camera motion
		double 	motion_cam_sigma_x_;
		double 	motion_cam_sigma_z_;
		double 	motion_cam_sigma_yaw_;
		double 	motion_cam_sigma_v_;
		double 	motion_cam_sigma_horizon_;

		double  perturb_cam_x_;
		double  perturb_cam_z_;
		double  perturb_cam_yaw_;
		double  perturb_cam_v_;
		double  perturb_cam_horizon_;
		///////////////////////////////////////////

		///////////////////////////////////////////
		// feature perturbation
		double  perturb_feat_x_;
		double  perturb_feat_z_;
		///////////////////////////////////////////

		double 	prob_enter_;
		double 	prob_stay_;
		double 	prob_feat_enter_;
		double 	prob_feat_stay_;
		double  prob_move_add_;
		double  prob_move_delete_;
		double  prob_move_stay_;
		double  prob_move_leave_;
		double  prob_move_update_;
		double  prob_move_interaction_flip_;
		double  prob_move_feat_stay_;
		double  prob_move_feat_leave_;
		double  prob_move_feat_update_;
		double  prob_move_cam_update_;

		int		max_gfeats_;
		bool	estimate_camera_;
		bool	interaction_on_;
		///////////////////////////////////////////
		// interaction parameters
		double 	repulsion_const_;
		double 	group_const_;
		double 	group_slope_;
		double 	group_threshold_;
		/*-------------------------
		-- Debug related functions
		---------------------------*/
	protected:
		// debug related
		void init_sampling_history() 
		{
			if(show_dbg_msg_ == false) return;

			memset(sample_count_, 0, sizeof(int) * MoveNums);
			memset(accept_count_, 0, sizeof(int) * MoveNums);
			memset(sample_tcost_, 0.0, sizeof(double) * MoveNums);

			struct timeval timeofday;
			gettimeofday(&timeofday,NULL);
			start_tic_ = last_tic_ = timeofday.tv_sec + timeofday.tv_usec * 1e-6;
		}

		void record_sampling_history(SampleInfo &info, bool accept)
		{
			if(show_dbg_msg_ == false) return;

			sample_count_[info.type_]++;
			if(accept) accept_count_[info.type_]++;

			struct timeval timeofday;
			gettimeofday(&timeofday,NULL);
			double ts = timeofday.tv_sec + timeofday.tv_usec * 1e-6;
			sample_tcost_[info.type_] += ts - last_tic_;
			last_tic_ = ts;
		}

		void print_sampling_history()
		{
			if(show_dbg_msg_ == false) return;

			float total_accept = 0.0, total_samples = 0.0;
			printf("=========================================================\n");
			for(uint i = 0; i < MoveNums; i++) {
				printf("Movetype%d, accepted %d, sampled %d, accept rate [%.03f], total time %.05f, avg time %.05f\n", i, 
								accept_count_[i], sample_count_[i], ((float) accept_count_[i])/((float)sample_count_[i]), 
								sample_tcost_[i], sample_tcost_[i] / sample_count_[i]);
				total_accept += (float)accept_count_[i];
				total_samples += (float)sample_count_[i];
			}

			struct timeval timeofday;
			gettimeofday(&timeofday,NULL);
			double ts = timeofday.tv_sec + timeofday.tv_usec * 1e-6;
			printf("Acceptance Ratio = %.03f, %.01f msecond past\n", total_accept / total_samples, (ts - start_tic_) * 1000);
			printf("=========================================================\n");
		}
		
		bool show_dbg_msg_;
		int sample_count_[MoveNums];
		int accept_count_[MoveNums];
		double sample_tcost_[MoveNums];
		double last_tic_;
		double start_tic_;
	};
};
#endif // RJMCMC_TRACKER_H_
