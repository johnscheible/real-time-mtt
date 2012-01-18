#ifndef _MCMC_SAMPLE_H_
#define _MCMC_SAMPLE_H_

#include <iostream>
#include <string.h>
#include <common/ped_state.h>
#include <common/gfeat_state.h>
#include <common/cam_state.h>
#include <common/util.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#define EPS 1e4*DBL_MIN// 1e-50

namespace people {
	using namespace std;

	class MCMCSample;

	typedef boost::shared_ptr<MCMCSample> MCMCSamplePtr;

	typedef enum {
		MoveNone = -1,
		MoveAdd,
		MoveDelete,
		MoveStay,
		MoveLeave,
		MoveUpdate,
		MoveCamUpdate,
		MoveFeatAdd,
		MoveFeatDelete,
		MoveFeatStay,
		MoveFeatLeave,
		MoveFeatUpdate, 
		MoveInteractionFlip,
		MoveNums,
	}MCMCMoveType;
	
	const char move_names[MoveNums][255] = {"Add", "Delete", "Stay", "Leave", "Update", "CamUpdate", "FeatAdd", "FeatDelete", "FeatStay", "FeatLeave", "FeatUpdate", "InteractionFlip"};

	class SampleInfo
	{
	public:
		SampleInfo():type_(MoveNone),idx_(0),idx2_(0),group_mode_(false) {};
		SampleInfo(SampleInfo &info) {
			type_ = info.type_; 
			idx_ = info.idx_;
			idx2_ = info.idx2_;
			state_ = info.state_;
			feat_state_ = info.feat_state_;
			cam_state_ = info.cam_state_;
			group_mode_ = info.group_mode_;
		};
		virtual ~SampleInfo() {};
		
		void print_info()
		{
			std::cout << "============================================" << std::endl;
			std::cout << "Move type : " << move_names[type_] << std::endl;
			std::cout << "idx : " << idx_ << std::endl;
			std::cout << "idx2 : " << idx2_ << std::endl;

			if(state_.get())  {
				std::cout << "target state : ";
				state_->print();
			}

			if(feat_state_.get()) {
				std::cout << "feat state : ";
				feat_state_->print();
			}

			if(cam_state_.get()) {
				std::cout << "cam state : ";
				cam_state_->print();
			}
			std::cout << "============================================" << std::endl;
		}

		MCMCMoveType 		type_; 	// move type
		unsigned int		idx_; 	// target idx
		unsigned int		idx2_;	// for interaction mode flip

		PeopleStatePtr	state_;	// target state
		GFeatStatePtr	feat_state_; // new feature state
		CamStatePtr		cam_state_; // new camera state
		bool			group_mode_; // invert group interaction?
	};

	class MCMCSample
	{
	public:
		MCMCSample()
		{
		};

		MCMCSample(const MCMCSample &sample)
		{
			states_ = sample.states_;
			exists_ = sample.exists_;
			feat_states_ = sample.feat_states_;
			valid_feats_ = sample.valid_feats_;
			cam_state_ = sample.cam_state_;
			group_interaction_ = sample.group_interaction_;
			lkhood_ = sample.lkhood_;
		};

		virtual ~MCMCSample()
		{
			states_.clear();
			exists_.clear();
			feat_states_.clear();
			valid_feats_.clear();
			group_interaction_.clear();
		};

		inline virtual void setInteractionMode(const std::vector< std::vector<bool> > &group_interaction)
		{
			group_interaction_ = group_interaction;
		};

		inline virtual void setGFeatStates(const std::vector<GFeatStatePtr> &feat_states, const std::vector<bool> &valid_feats)
		{
			feat_states_ = feat_states;
			valid_feats_ = valid_feats;
		};

		inline virtual void setStates(const std::vector<PeopleStatePtr> &states, const std::vector<bool> &exists, const double lkhood = 0.0)
		{
			states_ = states;
			exists_ = exists;
			lkhood_ = lkhood;
		};

		inline virtual void setCamState(const CamStatePtr cam_state)
		{
			cam_state_ = cam_state;
		};

		virtual MCMCSamplePtr setNewSample(const SampleInfo &info) 
		{
			MCMCSamplePtr ret = boost::make_shared<MCMCSample>(MCMCSample(*this));

			switch(info.type_) {
				case MoveAdd:
				case MoveStay:
				case MoveUpdate:
					my_assert(ret->getNumTargets() > (int)info.idx_);
					ret->states_[info.idx_] = info.state_;
					ret->exists_[info.idx_] = true;
					break;
				case MoveLeave:
				case MoveDelete:
					my_assert(ret->getNumTargets() > (int)info.idx_);
					ret->states_[info.idx_] = info.state_;
					ret->exists_[info.idx_] = false;
					break;
				case MoveCamUpdate:
					ret->cam_state_ = info.cam_state_;
					break;
				case MoveFeatAdd:
				case MoveFeatStay:
				case MoveFeatUpdate:
					my_assert(ret->getNumGFeats() > (int)info.idx_);
					ret->feat_states_[info.idx_] = info.feat_state_;
					ret->valid_feats_[info.idx_] = true;
					break;
				case MoveFeatLeave:
				case MoveFeatDelete:
					my_assert(ret->getNumGFeats() > (int)info.idx_);
					ret->feat_states_[info.idx_] = info.feat_state_;
					ret->valid_feats_[info.idx_] = false;
					break;
				case MoveInteractionFlip:
					(ret->group_interaction_[info.idx2_])[info.idx_] = info.group_mode_;
					break;
				default:
					my_assert(0);
			}
			return ret;
		};

		void removeTarget(int idx) 
		{
			dbg_assert();
			
			// remove a target
			std::vector<PeopleStatePtr>::iterator it1 = states_.begin() + idx;
			states_.erase(it1);

			std::vector<bool>::iterator it2 = exists_.begin() + idx;
			exists_.erase(it2);

			// remove all interaction related with the specified target
			std::vector<std::vector<bool> >::iterator it3 = group_interaction_.begin() + idx;
			group_interaction_.erase(it3);

			for(size_t i = 0; i < group_interaction_.size(); i++) {
				if(idx < (int)group_interaction_[i].size()) {
					std::vector<bool>::iterator it4 = group_interaction_[i].begin() + idx;
					group_interaction_[i].erase(it4);
				}
			}

			dbg_assert();
		}

		inline virtual bool getInteractionMode(int idx1, int idx2) {
			my_assert(idx1 < idx2);
			my_assert(idx2 < (int)group_interaction_.size());
			my_assert(idx1 <= (int)group_interaction_[idx2].size());

			return (group_interaction_[idx2])[idx1];
		};

		void removeFeature(int idx) 
		{
			std::vector<GFeatStatePtr>::iterator it1 = feat_states_.begin() + idx;
			feat_states_.erase(it1);

			std::vector<bool>::iterator it2 = valid_feats_.begin() + idx;
			valid_feats_.erase(it2);
		}
		
		inline virtual CamStatePtr getCamState() { return cam_state_; };

		inline virtual GFeatStatePtr getFeatState(int idx) {return feat_states_[idx];};
		inline virtual bool getFeatValidity(int idx) {return valid_feats_[idx];};
		inline virtual int getNumGFeats() { return feat_states_.size(); };

		inline virtual PeopleStatePtr getState(int idx) {return states_[idx];};
		inline virtual bool getExistance(int idx) {return exists_[idx];};
		inline virtual int getNumTargets() { return states_.size(); };

		void dbg_assert() {
			my_assert(states_.size() == exists_.size()); 
			my_assert(feat_states_.size() == valid_feats_.size()); 
			my_assert(states_.size() == group_interaction_.size()); 
			for(size_t i = 0; i < states_.size(); i++) {
				my_assert(group_interaction_[i].size() == i); 
			}
		}

		void print() {
			cam_state_->print();
			for(size_t i = 0; i < states_.size(); i++) {
				std::cout << "target " << i << " : ";
				if(exists_[i]) states_[i]->print();
				else std::cout << "none" << std::endl;
			}
			for(size_t i = 0; i < feat_states_.size(); i++) {
				std::cout << "feat " << i << " : ";
				if(valid_feats_[i]) feat_states_[i]->print();
				else std::cout << "none" << std::endl;
			}
		}
	protected:
		std::vector<PeopleStatePtr> 	states_;
		std::vector<bool>				exists_;
		std::vector< std::vector<bool> >	group_interaction_; // {n * (n - 1) / 2} number of variables. upper triangle of n by n matrix (not including diagonal).

		std::vector<GFeatStatePtr>		feat_states_;
		std::vector<bool>				valid_feats_;
		
		CamStatePtr						cam_state_;

		double 							lkhood_;
	};

	class TargetDist
	{
	public:
#ifdef VEL_STATE
		TargetDist():nframes_(1),sigma_x_(0.8),sigma_y_(0.8),sigma_z_(0.4),sigma_vx_(0.5),sigma_vy_(0.5),sigma_vz_(0.1){};
#else
		TargetDist():nframes_(1),sigma_x_(0.8),sigma_y_(0.8),sigma_z_(0.4){};
#endif
		virtual ~TargetDist(){};

#ifdef VEL_STATE
		void setParameters(const double sigma_x, const double sigma_y, const double sigma_z, const double sigma_vx, const double sigma_vy, const double sigma_vz)
		{
			sigma_x_ = sigma_x;
			sigma_y_ = sigma_y;
			sigma_z_ = sigma_z;
			sigma_vx_ = sigma_vx;
			sigma_vy_ = sigma_vy;
			sigma_vz_ = sigma_vz;
		};
#else
		void setParameters(const double sigma_x, const double sigma_y, const double sigma_z)
		{
			sigma_x_ = sigma_x;
			sigma_y_ = sigma_y;
			sigma_z_ = sigma_z;
		};
#endif
		virtual void setSamples(const std::vector<PeopleStatePtr> &samples, int maxSamples)
		{
			samples_ = samples;
			updateStatistics(maxSamples);
		};
		
		inline std::vector<PeopleStatePtr> &getStates() {return samples_;}
		inline PeopleStatePtr getMean() { return boost::make_shared<PeopleState>(mean_); };
		
		inline int getFrames() {return nframes_;};
		inline void setFrames(int frames) {nframes_ = frames;};

		virtual PeopleStatePtr drawSample(const double timestamp)
		{
			PeopleStatePtr ret;
			int idx = floor(rng_.uniform((double)0.0f, (double)samples_.size() - EPS));
#ifdef VEL_STATE
			assert(0);

			ret = samples_[idx]->clone();
			double dt = timestamp - ret->getTS(); // uncertainty over time
			double vel_factor = get_vel_factor(nframes_);

			// not gonna allow x_
			double temp;
			temp = ret->getX() + rng_.gaussian(sigma_x_ * dt);			ret->setX(temp);
			temp = ret->getY() + rng_.gaussian(sigma_y_ * dt); 			ret->setY(temp);
			temp = ret->getZ() + rng_.gaussian(sigma_z_ * dt); 			ret->setZ(temp);
			temp = ret->getVX() + rng_.gaussian(sigma_vx_ * dt * vel_factor);			ret->setVX(temp);
			temp = ret->getVY() + rng_.gaussian(sigma_vy_ * dt * vel_factor); 		ret->setVY(temp);
			temp = ret->getVZ() + rng_.gaussian(sigma_vz_ * dt * vel_factor); 		ret->setVZ(temp);
			
			ret = ret->predict(timestamp);
#else
			ret = samples_[idx]->clone();
			double dt = timestamp - ret->getTS(); // uncertainty over time
			double temp;
			temp = ret->getX() + rng_.gaussian(sigma_x_ * dt);			ret->setX(temp);
			temp = ret->getY() + rng_.gaussian(sigma_y_ * dt); 			ret->setY(temp);
			temp = ret->getZ() + rng_.gaussian(sigma_z_ * dt); 			ret->setZ(temp);
			ret->setTS(timestamp);
#endif
			return ret;
		};

		virtual double getSampleProbability(PeopleStatePtr state, double timestamp)
		{
			double ret = 0.0;
#ifdef VEL_STATE
			assert(0);
#else
			double dt = timestamp - samples_[0]->getTS(); // uncertainty over time
			for(int i = 0; i < (int)samples_.size(); i++)
			{
				ret += (gaussian_prob(state->getX(), samples_[i]->getX(), sigma_x_ * dt)
							* gaussian_prob(state->getY(), samples_[i]->getY(), sigma_y_ * dt)
							* gaussian_prob(state->getZ(), samples_[i]->getZ(), sigma_z_ * dt)) / samples_.size();
			}
			ret += EPS;
#endif
			return ret;
		};
	protected:
		void updateStatistics(int maxSamples)
		{
			int N = samples_.size();
			if(N == 0)
			{
				mean_.setX(0.0);
				mean_.setY(0.0);
				mean_.setZ(0.0);
#ifdef VEL_STATE
				mean_.setVX(0.0);
				mean_.setVY(0.0);
				mean_.setVZ(0.0);
#endif
				mean_.setConfidence(0.0);
				return;
			}

			mean_.setConfidence((double)N / maxSamples);
			mean_.setTS(samples_[0]->getTS());
			
			double x = 0.0, y = 0.0, z = 0.0;
#ifdef VEL_STATE
			double vx = 0.0, vy = 0.0, vz = 0.0;
#endif
			for(int i = 0; i < N; i++)
			{
				x += samples_[i]->getX(); y += samples_[i]->getY(); z += samples_[i]->getZ();
#ifdef VEL_STATE
				vx += samples_[i]->getVX(); vy += samples_[i]->getVY(); vz += samples_[i]->getVZ();
#endif
			}

			mean_.setX(x / N); 	mean_.setY(y / N); mean_.setZ(z / N);
#ifdef VEL_STATE
			mean_.setVX(vx / N); 	mean_.setVY(vy / N); mean_.setVZ(vz / N);
#endif
		};
	protected:
		// number of frames tracked
		int						nframes_;
		// random value generator
		cv::RNG					rng_;
		// approx dist : samples from MCMC
		std::vector<PeopleStatePtr> samples_;

		// motion uncertainty - parameter of tracking
		double sigma_x_;
		double sigma_y_;
		double sigma_z_;
#ifdef VEL_STATE
		double sigma_vx_;
		double sigma_vy_;
		double sigma_vz_;
#endif
		// mean and variance of the target
		PeopleState			mean_;
	};

	typedef boost::shared_ptr<TargetDist> TargetDistPtr;

	class FeatureDist
	{
	public:
		FeatureDist():nframes_(1){};
		virtual ~FeatureDist(){};
		virtual void insertSample(GFeatStatePtr feat, bool valid) {
			if(valid)	{
				feats_.push_back(feat);
				my_assert(feat.get());
			}
		};

		void updateStatistics(int maxSamples)
		{
			int N = feats_.size();
			if(N == 0)
			{
				mean_.setX(0.0); mean_.setY(0.0); mean_.setZ(0.0);
				mean_.setConfidence(0.0);
				return;
			}
			mean_.setConfidence((double)N / maxSamples);
			mean_.setTS(feats_[0]->getTS());
			
			double x = 0.0, y = 0.0, z = 0.0;
			for(int i = 0; i < N; i++)
			{
				x += feats_[i]->getX(); y += feats_[i]->getY(); z += feats_[i]->getZ();
			}
			mean_.setX(x / N); 	mean_.setY(y / N); mean_.setZ(z / N);
		};

		inline GFeatStatePtr getMean() {return boost::make_shared<GFeatState>(mean_);};

		virtual GFeatStatePtr drawSample(const double timestamp)
		{
			GFeatStatePtr ret;

			assert(feats_.size() > 0);

			int idx = floor(rng_.uniform((double)0.0f, (double)feats_.size() - EPS));
			ret = feats_[idx]->clone();
			ret->setTS(timestamp);
			return ret;
		};

		virtual double getSampleProbability(GFeatStatePtr state, double timestamp)
		{
#ifdef MYDEBUG
			// should be same with at least one sample
#endif
			return (double)1 / feats_.size();
		};

		inline int getFrames() {return nframes_;};
		inline void setFrames(int frames) {nframes_ = frames;};
		
		void print_all_features() 
		{
			std::cout << "total " << feats_.size() << " number of features." << std::endl;
			for(size_t i = 0; i < feats_.size(); i++) {
				std::cout << i << "th ";
				feats_[i]->print();
			}
		}

		inline std::vector<GFeatStatePtr> getAllSamples(){return feats_;}
	protected:
		int							nframes_;
		std::vector<GFeatStatePtr>	feats_;
		cv::RNG						rng_;
		GFeatState					mean_;
	};

	typedef boost::shared_ptr<FeatureDist> FeatureDistPtr;

	class PosteriorDist
	{
	public:
#ifdef VEL_STATE
		PosteriorDist():sigma_x_(0.01),sigma_y_(0.01),sigma_z_(0.01),sigma_vx_(5),sigma_vy_(5),sigma_vz_(1)
		{
			initialize();
		}

		PosteriorDist(double sigma_x, double sigma_y, double sigma_z, double sigma_vx, double sigma_vy, double sigma_vz):sigma_x_(sigma_x),sigma_y_(sigma_y),sigma_z_(sigma_z),sigma_vx_(sigma_vx),sigma_vy_(sigma_vy),sigma_vz_(sigma_vz) 
		{
			initialize();
		}
#else
		PosteriorDist():sigma_x_(1.0),sigma_y_(1.0),sigma_z_(0.5) 
		{
			initialize();
		}

		PosteriorDist(double sigma_x, double sigma_y, double sigma_z):sigma_x_(sigma_x),sigma_y_(sigma_y),sigma_z_(sigma_z) 
		{
			initialize();
		}
#endif
		virtual ~PosteriorDist() {};
		
		void initialize()
		{
			MCMCSamplePtr sample = boost::make_shared<MCMCSample>(MCMCSample());
			std::vector<MCMCSamplePtr> samples;
			samples.push_back(sample);
			setSamples(samples, 0, 1);
		}
#ifdef VEL_STATE
		void setParameters(double sigma_x, double sigma_y, double sigma_z, double sigma_vx, double sigma_vy, double sigma_vz) 
		{
			sigma_x_ = sigma_x;
			sigma_y_ = sigma_y;
			sigma_z_ = sigma_z;
			sigma_vx_ = sigma_vx;
			sigma_vy_ = sigma_vy;
			sigma_vz_ = sigma_vz;
		}
#else
		void setParameters(double sigma_x, double sigma_y, double sigma_z) 
		{
			sigma_x_ = sigma_x;
			sigma_y_ = sigma_y;
			sigma_z_ = sigma_z;
		}
#endif
		void setSamples(const std::vector<MCMCSamplePtr> &samples, int burnin, int thinning) 
		{
			int maxSamples = floor((float)(samples.size() - burnin) / thinning);

			std::vector<int> tframes;
			for(size_t i = 0; i < targets_.size(); i++) {
				tframes.push_back(targets_[i]->getFrames() + 1);
			}

			std::vector<int> fframes;
			for(size_t i = 0; i < feat_dists_.size(); i++) {
				fframes.push_back(feat_dists_[i]->getFrames() + 1);
			}

			samples_.clear();
			targets_.clear();
			feat_dists_.clear();

			for(int i = burnin; i < (int)samples.size(); i+= thinning)
				samples_.push_back(samples[i]);
	
			double ts = .0;
			for(int i = 0; i < samples_[0]->getNumTargets(); i++)
			{
				TargetDistPtr target = boost::make_shared<TargetDist>(TargetDist());
#ifdef VEL_STATE
				target->setParameters(sigma_x_, sigma_y_, sigma_z_, sigma_vx_, sigma_vy_, sigma_vz_);
#else
				target->setParameters(sigma_x_, sigma_y_, sigma_z_);
#endif
				if(i < (int)tframes.size()) {
					my_assert(tframes[i] > 0);
					target->setFrames(tframes[i]);
				}

				std::vector<PeopleStatePtr> states;
				for(int j = 0; j < (int)samples_.size(); j++) {
					if(samples_[j]->getExistance(i)) {
						states.push_back(samples_[j]->getState(i));
#if 1 
						// must be satisfied!!!!!!!!!
						if(ts > 0) {
							my_assert(ts == samples_[j]->getState(i)->getTS());
						}
						else {
							ts = samples_[j]->getState(i)->getTS();
						}
#endif
					}
				}
				target->setSamples(states, maxSamples);
				targets_.push_back(target);
			}

			for(int i = 0; i < samples_[0]->getNumGFeats(); i++) {
				FeatureDistPtr feat = boost::make_shared<FeatureDist>(FeatureDist());
				if(i < (int)fframes.size()) {
					my_assert(fframes[i] > 0);
					feat->setFrames(fframes[i]);
				}
				for(int j = 0; j < (int)samples_.size(); j++) {
					feat->insertSample(samples_[j]->getFeatState(i), samples_[j]->getFeatValidity(i));
				}
				feat->updateStatistics(maxSamples);
				feat_dists_.push_back(feat);
			}
#ifdef VEL_STATE
			// fake sample to put zero velocity prior for static targets
			MCMCSamplePtr sample = boost::make_shared<MCMCSample>(MCMCSample());
			std::vector<PeopleStatePtr> states;
			std::vector<bool> exists;

			for(int i = 0; i < (int)targets_.size(); i++) {
				PeopleStatePtr state = targets_[i]->getMean()->clone();
				state->setVX(0.0);
				state->setVY(0.0); 
				state->setVZ(0.0);

				states.push_back(state);
				exists.push_back(true);
			}

			sample->setStates(states, exists);
			std::vector< std::vector<bool> > group_interaction;
			for(unsigned int i = 0; i < targets_.size(); i++) {
				std::vector<bool> one_col;
				for(unsigned int j = 0; j < i; j++) {
					one_col.push_back(false); // by default no group interaction
				}
				group_interaction.push_back(one_col);
			}
			sample->setInteractionMode(group_interaction);

#ifdef CAM_EST
			if(samples_[0]->getCamState().get()) { // if it's not a fake initialization
				CamStatePtr cam = getMeanCamera();
				sample->setCamState(cam);
			}

			std::vector<bool> feat_validities;
			std::vector<GFeatStatePtr> feat_states;
			for(int i = 0; i < getNumFeats(); i++) {
				double val = getMeanFeatValidity(i);
				GFeatStatePtr feat = getMeanFeature(i);

				feat_validities.push_back(val > 0.5);
				feat_states.push_back(feat);
			}
			sample->setGFeatStates(feat_states, feat_validities);
#endif
			samples_.push_back(sample);	
#endif

#ifdef _DEBUG
			std::cout << "sampling done" << std::endl;
#endif
		};
		
		double getTimeStamp() 
		{
			double ts = .0;

			for(int i = 0; i < (int)samples_.size(); i++) {
				CamStatePtr cam = samples_[i]->getCamState();
				if(cam.get() == NULL) continue;
				if(ts > 0) {
					my_assert(ts == cam->getTS());
				}
				else {
					ts = cam->getTS();
				}
			}

			return ts;
		};

		inline MCMCSamplePtr getSample(int idx) {return samples_[idx];};
		inline TargetDistPtr getTarget(int idx){return targets_[idx];}
		inline FeatureDistPtr getFeatureDist(int idx){return feat_dists_[idx];}

		inline int getNumTargets() {return targets_.size();};
		inline int getNumSamples() {return samples_.size();};
		inline int getNumFeats() {	return feat_dists_.size(); };

		GFeatStatePtr drawFeatSample(int idx, double timestamp)
		{
			return feat_dists_[idx]->drawSample(timestamp);
		}

		double getMeanInteractionMode(int min_idx, int max_idx)
		{
			double ret = 0.0;
			my_assert(min_idx < max_idx);
			for(size_t i = 0; i < samples_.size(); i++) {
				ret += (double)samples_[i]->getInteractionMode(min_idx, max_idx);
			}
			ret /= (double)samples_.size();
			return ret;
		};

		CamStatePtr getMeanCamera() 
		{
			CamStatePtr state = boost::make_shared<CamState>(CamState());

			if(samples_[0]->getCamState().get() == NULL) {
				// no camera information exists. must be the first frame
				return samples_[0]->getCamState();
			}

			double x = 0.0, y = 0.0, z = 0.0;
			double v = 0.0, yaw = 0.0, horizon = 0.0;
			double f = 0.0, xcenter = 0.0;
			state->setTS(samples_[0]->getCamState()->getTS());

			for(size_t i = 0; i < samples_.size(); i++) {
				CamStatePtr cam_sample = samples_[i]->getCamState();
				x += cam_sample->getX() / samples_.size();
				y += cam_sample->getY() / samples_.size();
				z += cam_sample->getZ() / samples_.size();
				v += cam_sample->getV() / samples_.size();
				yaw += cam_sample->getYaw() / samples_.size();
				horizon += cam_sample->getHorizon() / samples_.size();
				f += cam_sample->getFocal() / samples_.size();
				xcenter += cam_sample->getXcenter() / samples_.size();
				
				my_assert(state->getTS() == cam_sample->getTS());
			}

			state->setX(x); state->setY(y); state->setZ(z);
			state->setV(v); state->setYaw(yaw); state->setHorizon(horizon);
			state->setFocal(f); state->setXcenter(xcenter);

			return state;
		}
		
		double getMeanFeatValidity(int idx)
		{
			return feat_dists_[idx]->getMean()->getConfidence();
#if 0
			double ret = 0.0;
			for(size_t i = 0; i < samples_.size(); i++) {
				ret += (double)samples_[i]->getFeatValidity(idx) / samples_.size();
			}
			return ret;
#endif
		};

		GFeatStatePtr getMeanFeature(int idx)
		{
			return feat_dists_[idx]->getMean();
#if 0
			GFeatStatePtr state = boost::make_shared<GFeatState>(GFeatState());
			int cnt = 0;
			for(size_t i = 0; i < samples_.size(); i++) {
				if(samples_[i]->getFeatValidity(idx)) {
					GFeatStatePtr feat_sample = samples_[i]->getFeatState(idx);
					state->x_ += feat_sample->x_;
					state->y_ += feat_sample->y_;
					state->z_ += feat_sample->z_;
					cnt++;
				}
			}
			state->x_ /= cnt;
			state->y_ /= cnt;
			state->z_ /= cnt;
			return state;
#endif
		};

		PeopleStatePtr drawSample(int idx, double timestamp)
		{
			return targets_[idx]->drawSample(timestamp);
		};

		double getSampleProbability(PeopleStatePtr state, int idx, double timestamp)
		{
			return targets_[idx]->getSampleProbability(state, timestamp);
		};

		inline std::vector<TargetDistPtr> getTargetDists(){return targets_;}
		inline std::vector<FeatureDistPtr> getFeatureDists(){return feat_dists_;}

		void removeTarget(int idx)
		{
#ifdef _DEBUG
			std::cout << "removing target" << idx << std::endl;
#endif
			my_assert(samples_.size() > 0);
			std::vector<TargetDistPtr>::iterator it = targets_.begin() + idx;
			targets_.erase(it);

			MCMCSample* last = NULL;
			for(size_t i = 0; i < samples_.size(); i++) {
				// necessary to check since we are using the shared pointer 
				// and there is no unique index for targets.... 
				// not so elegant...
				if(last != samples_[i].get())
					samples_[i]->removeTarget(idx);

				last = samples_[i].get();
				my_assert(targets_.size() == (size_t)samples_[i]->getNumTargets());
			}
		}
	
		void removeFeature(int idx)
		{
			my_assert(samples_.size() > 0);
			std::vector<FeatureDistPtr>::iterator it = feat_dists_.begin() + idx;
			feat_dists_.erase(it);

			MCMCSample* last = NULL;
			for(size_t i = 0; i < samples_.size(); i++) {
				if(last != samples_[i].get())
					samples_[i]->removeFeature(idx);

				last = samples_[i].get();
				my_assert(feat_dists_.size() == (size_t)samples_[i]->getNumGFeats());
			}
		}

		void print_target_summary()
		{
			std::cout << "Camera Mean!" << std::endl;
			std::cout << "====================" << std::endl;
			getMeanCamera()->print();
			std::cout << "====================" << std::endl;

			std::cout << "Target Summary!" << std::endl;
			for(size_t i = 0; i < targets_.size(); i++) {
				std::cout << "target[" << i << "]===" << std::endl;
				targets_[i]->getMean()->print();
				std::cout << "tracked for " << targets_[i]->getFrames() << " frames" << std::endl;
				std::cout << "====================" << std::endl;
			}

			std::cout << "Interactions " << std::endl;
			for(size_t i = 0; i < targets_.size(); i++) {
				for(size_t j = 0; j < targets_.size(); j++) {
					if(j <= i)
						std::cout << "- ";
					else
						std::cout << std::setprecision(3) << getMeanInteractionMode(i, j) << " ";
				}
				std::cout << std::endl;
			}
#if 0
			std::cout << "Feature Summary!" << std::endl;
			for(size_t i = 0; i < feat_dists_.size(); i++) {
				std::cout << "feature[" << i << "]===" << std::endl;
				feat_dists_[i]->getMean()->print();
				std::cout << "tracked for " << feat_dists_[i]->getFrames() << " frames" << std::endl;
				std::cout << "====================" << std::endl;
			}
#endif
		}
	protected:
		// random value generator
		cv::RNG					rng_;

		std::vector<MCMCSamplePtr> samples_;
		std::vector<TargetDistPtr> targets_;
		std::vector<FeatureDistPtr> feat_dists_;
		// motion uncertainty - parameter of tracking
		double sigma_x_;
		double sigma_y_;
		double sigma_z_;
#ifdef VEL_STATE
		double sigma_vx_;
		double sigma_vy_;
		double sigma_vz_;
#endif
	};

	typedef boost::shared_ptr<PosteriorDist> PosteriorDistPtr;
}; // NAMESPACE
#endif // _MCMC_SAMPLE_H_
