/*
 * Software License Agreement (BSD License)
 * 
 * Copyright (c)  2012, Wongun Choi
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies, 
 * either expressed or implied, of the FreeBSD Project.
 */

#ifndef _MCMC_SAMPLE_H_
#define _MCMC_SAMPLE_H_

#include <iostream>
#include <string.h>
#include <common/states.h>
#include <common/ped_state.h>
#include <common/gfeat_state.h>
#include <common/util.h>
#include <boost/shared_ptr.hpp>

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

			obj_state_ = info.obj_state_;
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

			if(obj_state_.get())  {
				std::cout << "target state : ";
				obj_state_->print();
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

		ObjectStatePtr		obj_state_;	// target state
		FeatureStatePtr		feat_state_; // new feature state
		CameraStatePtr		cam_state_; // new camera state
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
			obj_states_ = sample.obj_states_;
			obj_exists_ = sample.obj_exists_;
			feat_states_ = sample.feat_states_;
			valid_feats_ = sample.valid_feats_;
			cam_state_ = sample.cam_state_;
			group_interaction_ = sample.group_interaction_;
			lkhood_ = sample.lkhood_;
		};

		virtual ~MCMCSample()
		{
			obj_states_.clear();
			obj_exists_.clear();
			feat_states_.clear();
			valid_feats_.clear();
			group_interaction_.clear();
		};

		inline virtual void setInteractionMode(const std::vector< std::vector<bool> > &group_interaction)
		{
			group_interaction_ = group_interaction;
		};

		inline virtual void setFeatureStates(const std::vector<FeatureStatePtr> &feat_states, const std::vector<bool> &valid_feats)
		{
			feat_states_ = feat_states;
			valid_feats_ = valid_feats;
		};

		inline virtual void setObjectStates(const std::vector<ObjectStatePtr> &states, const std::vector<bool> &exists, const double lkhood = 0.0)
		{
			obj_states_ = states;
			obj_exists_ = exists;
			lkhood_ = lkhood;
		};

		inline virtual void setCameraState(const CameraStatePtr cam_state)
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
					ret->obj_states_[info.idx_] = info.obj_state_;
					ret->obj_exists_[info.idx_] = true;
					break;
				case MoveLeave:
				case MoveDelete:
					my_assert(ret->getNumTargets() > (int)info.idx_);
					ret->obj_states_[info.idx_] = info.obj_state_;
					ret->obj_exists_[info.idx_] = false;
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
			std::vector<ObjectStatePtr>::iterator it1 = obj_states_.begin() + idx;
			obj_states_.erase(it1);

			std::vector<bool>::iterator it2 = obj_exists_.begin() + idx;
			obj_exists_.erase(it2);

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
			std::vector<FeatureStatePtr>::iterator it1 = feat_states_.begin() + idx;
			feat_states_.erase(it1);

			std::vector<bool>::iterator it2 = valid_feats_.begin() + idx;
			valid_feats_.erase(it2);
		}
		
		inline virtual CameraStatePtr getCameraState() { return cam_state_; };

		inline virtual FeatureStatePtr getFeatureState(int idx) {return feat_states_[idx];};
		inline virtual bool getFeatureValidity(int idx) {return valid_feats_[idx];};
		inline virtual int getNumFeatures() { return feat_states_.size(); };

		inline virtual ObjectStatePtr getObjectState(int idx) {return obj_states_[idx];};
		inline virtual bool getObjectExistance(int idx) {return obj_exists_[idx];};
		inline virtual int getNumObjects() { return obj_states_.size(); };

		void dbg_assert() {
			my_assert(obj_states_.size() == obj_exists_.size()); 
			my_assert(feat_states_.size() == valid_feats_.size()); 
			my_assert(obj_states_.size() == group_interaction_.size()); 
			for(size_t i = 0; i < obj_states_.size(); i++) {
				my_assert(group_interaction_[i].size() == i); 
			}
		}

		void print() {
			cam_state_->print();
			for(size_t i = 0; i < obj_states_.size(); i++) {
				std::cout << "target " << i << " : ";
				if(obj_exists_[i]) obj_states_[i]->print();
				else std::cout << "none" << std::endl;
			}
			for(size_t i = 0; i < feat_states_.size(); i++) {
				std::cout << "feat " << i << " : ";
				if(valid_feats_[i]) feat_states_[i]->print();
				else std::cout << "none" << std::endl;
			}
		}
	protected:
		std::vector<ObjectStatePtr> 			obj_states_;
		std::vector<bool>									obj_exists_;
		std::vector< std::vector<bool> >	group_interaction_; // {n * (n - 1) / 2} number of variables. upper triangle of n by n matrix (not including diagonal).

		std::vector<FeatureStatePtr>			feat_states_;
		std::vector<bool>									valid_feats_;
		
		CameraStatePtr										cam_state_;

		double 														lkhood_;
	};

	class TargetDist
	{
	public:
		TargetDist():nframes_(1)
		{
			motion_params_.push_back(0.8);
			motion_params_.push_back(0.8);
			motion_params_.push_back(0.4);
			motion_params_.push_back(0.5);
			motion_params_.push_back(0.5);
			motion_params_.push_back(0.1);
		};

		TargetDist(const std::vector<double> &motion_params):nframes_(1)
		{
			motion_params_ = motion_params;
		}

		virtual ~TargetDist(){};

		void setParameters(const std::vector<double> &motion_params) {
			motion_params_ = motion_params;
		}

		virtual void setSamples(const std::vector<ObjectStatePtr> &samples, int maxSamples)
		{
			samples_ = samples;
			updateStatistics(maxSamples);
		}
		
		inline std::vector<ObjectStatePtr> &getStates() { return samples_; }
		inline ObjectStatePtr getMean() { return mean_; };
		
		inline int getFrames() { return nframes_; };
		inline void setFrames(int frames) { nframes_ = frames; };

		virtual ObjectStatePtr drawSample(const double timesec)
		{
			int idx = floor(g_rng.uniform((double)0.0f, (double)samples_.size() - EPS));
			assert(motion_params_[0] > 0.0);
			return samples_[idx]->drawSample(timesec, motion_params_);
		};

		virtual double getSampleProbability(ObjectStatePtr state, double timesec)
		{
			double ret = 0.0;
			for(int i = 0; i < (int)samples_.size(); i++)
				ret += samples_[i]->computeLogPrior(state, timesec, motion_params_);

			ret += EPS;	ret /= samples_.size();
			return ret;
		};
	protected:
		void updateStatistics(int maxSamples)
		{
			size_t N = samples_.size();

			if(N == 0) {
				mean_ = ObjectStatePtr(new ObjectStateVel);
				for(size_t i = 0; i < mean_->numElement(); i++) {
					mean_->setElement(i, 0.0);
				}
				mean_->setTS(0.0);
				mean_->setConfidence(0.0);
				return;
			}
			
			// initialize memory, if empty
			if(mean_.get() == NULL) {
				mean_ = samples_[0]->clone();
			}

			mean_->setConfidence((double)N / maxSamples);
			mean_->setTS(samples_[0]->getTS());
			for(size_t i = 0; i < mean_->numElement(); i++) {
				double temp = 0.0;
				for(size_t j = 0; j < N; j++) {
					temp += samples_[j]->getElement(i);
				}
				temp /= N;
				mean_->setElement(i, temp);
			}
		};
	protected:
		// number of frames tracked
		int													nframes_;
		// approx dist : samples from MCMC
		std::vector<ObjectStatePtr> samples_;
		// motion uncertainty - parameter of tracking
		std::vector<double>					motion_params_;
		// mean and variance of the target
		ObjectStatePtr							mean_;
	};

	typedef boost::shared_ptr<TargetDist> TargetDistPtr;

	class FeatureDist
	{
	public:
		FeatureDist():nframes_(1){
			motion_params_.push_back(0.1);
			motion_params_.push_back(0.1);
			motion_params_.push_back(0.1);
		};

		FeatureDist(const std::vector<double> &motion_params):nframes_(1)
		{
			motion_params_ = motion_params;
		}

		virtual ~FeatureDist(){};

		void setParameters(const std::vector<double> &motion_params) {
			motion_params_ = motion_params;
		}

		virtual void insertSample(FeatureStatePtr feat, bool valid) {
			if(valid)	{
				feats_.push_back(feat);
				my_assert(feat.get());
			}
		};

		void updateStatistics(int maxSamples)
		{
			size_t N = feats_.size();
			if(N == 0) {
				mean_ = FeatureStatePtr(new StaticFeatureState);
				for(size_t i = 0; i < mean_->numElement(); i++) {
					mean_->setElement(i, 0.0);
				}
				mean_->setTS(0.0);
				mean_->setConfidence(0.0);
				return;
			}

			// initialize memory, if empty
			if(mean_.get() == NULL) {
				mean_ = feats_[0]->clone();
			}

			mean_->setConfidence((double)N / maxSamples);
			mean_->setTS(feats_[0]->getTS());
			for(size_t i = 0; i < mean_->numElement(); i++) {
				double temp = 0.0;
				for(size_t j = 0; j < N; j++) {
					temp += feats_[j]->getElement(i);
				}
				temp /= N;
				mean_->setElement(i, temp);
			}
		};

		inline FeatureStatePtr getMean() {return mean_;};

		virtual FeatureStatePtr drawSample(const double timestamp)
		{
			FeatureStatePtr ret;
			assert(feats_.size() > 0);
			int idx = floor(g_rng.uniform((double)0.0f, (double)feats_.size() - EPS));
			ret = feats_[idx]->clone();
			ret->setTS(timestamp);
			return ret;
		};

		virtual double getSampleProbability(FeatureStatePtr state, double timestamp)
		{
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

		inline std::vector<FeatureStatePtr> getAllSamples(){ return feats_; }
	protected:
		int														nframes_;
		std::vector<FeatureStatePtr>	feats_;
		// motion uncertainty - parameter of tracking
		std::vector<double>						motion_params_;
		// mean and variance of the feature
		FeatureStatePtr								mean_;
	};

	typedef boost::shared_ptr<FeatureDist> FeatureDistPtr;

	class PosteriorDist
	{
	public:
		PosteriorDist()
		{
			initialize();
		}

		PosteriorDist(const std::vector<double> &obj_motion_params, const std::vector<double> &feat_motion_params, const std::vector<double> &cam_motion_params)
		{
			obj_motion_params_ = obj_motion_params;
			feat_motion_params_ = feat_motion_params;
			cam_motion_params_ = cam_motion_params;

			initialize();
		}

		virtual ~PosteriorDist() {};
		
		void initialize()
		{
			MCMCSamplePtr sample(new MCMCSample());
			std::vector<MCMCSamplePtr> samples;
			samples.push_back(sample);
			setSamples(samples, 0, 1);
		}

		void setObjectParameters(const std::vector<double> &motion_params) {
			obj_motion_params_ = motion_params;
		}

		void setFeatureParameters(const std::vector<double> &motion_params) {
			feat_motion_params_ = motion_params;
		}

		void setCameraParameters(const std::vector<double> &motion_params) {
			cam_motion_params_ = motion_params;
		}

		void setSamples(const std::vector<MCMCSamplePtr> &samples, int burnin, int thinning)
		{
			int maxSamples = floor((float)(samples.size() - burnin) / thinning);

			// copy previous information/frames to be tracked
			std::vector<int> tframes;
			for(size_t i = 0; i < targets_.size(); i++) {
				tframes.push_back(targets_[i]->getFrames() + 1);
			}
			std::vector<int> fframes;
			for(size_t i = 0; i < feat_dists_.size(); i++) {
				fframes.push_back(feat_dists_[i]->getFrames() + 1);
			}

			// clear samples
			samples_.clear();
			targets_.clear();
			feat_dists_.clear();

			// choose samples - burnin/thinning
			for(int i = burnin; i < (int)samples.size(); i+= thinning)
				samples_.push_back(samples[i]);
	
			// set object samples
			double ts = .0;
			for(int i = 0; i < samples_[0]->getNumObjects(); i++)
			{
				TargetDistPtr target(new TargetDist(obj_motion_params_));
				if(i < (int)tframes.size()) {
					my_assert(tframes[i] > 0);
					target->setFrames(tframes[i]);
				}

				std::vector<ObjectStatePtr> states;
				for(int j = 0; j < (int)samples_.size(); j++) {
					if(samples_[j]->getObjectExistance(i)) {
						states.push_back(samples_[j]->getObjectState(i));
						// must be satisfied!!!!!!!!!
						if(ts > 0) {
							my_assert(ts == samples_[j]->getObjectState(i)->getTS());
						}
						else {
							ts = samples_[j]->getObjectState(i)->getTS();
						}
					}
				}
				target->setSamples(states, maxSamples);
				targets_.push_back(target);
			}

			// set feature samples
			for(int i = 0; i < samples_[0]->getNumFeatures(); i++) {
				FeatureDistPtr feat(new FeatureDist(feat_motion_params_));
				if(i < (int)fframes.size()) {
					my_assert(fframes[i] > 0);
					feat->setFrames(fframes[i]);
				}
				for(int j = 0; j < (int)samples_.size(); j++) {
					feat->insertSample(samples_[j]->getFeatureState(i), samples_[j]->getFeatureValidity(i));
				}
				feat->updateStatistics(maxSamples);
				feat_dists_.push_back(feat);
			}
#ifdef _DEBUG
			std::cout << "sampling done" << std::endl;
#endif
		};
		
		double getTimeStamp() 
		{
			double ts = .0;
			for(int i = 0; i < (int)samples_.size(); i++) {
				CameraStatePtr cam = samples_[i]->getCameraState();
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
		inline TargetDistPtr getTargetDist(int idx){return targets_[idx];}
		inline FeatureDistPtr getFeatureDist(int idx){return feat_dists_[idx];}

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

		inline int getNumTargets() {return targets_.size();};
		inline int getNumFeatures() {	return feat_dists_.size(); };
		inline int getNumSamples() {return samples_.size();};

		CameraStatePtr getMeanCamera() 
		{
			if(samples_[0]->getCameraState().get() == NULL) {
				// no camera information exists. must be the first frame
				return samples_[0]->getCameraState();
			}
			CameraStatePtr state = samples_[0]->getCameraState()->clone(); 
			// boost::make_shared<CamState>(CamState());
			state->setTS(samples_[0]->getCameraState()->getTS());

			for(size_t i = 0; i < state->numElement(); i++) {
				double temp = 0.0;
				for(size_t j = 0; j < samples_.size(); j++) {
					temp += samples_[j]->getCameraState()->getElement(i);
				}
				temp /= samples_.size();
				state->setElement(i, temp);
			}

			return state;
		}
		
		double getMeanFeatureValidity(int idx)
		{
			return feat_dists_[idx]->getMean()->getConfidence();
		}

		FeatureStatePtr getMeanFeature(int idx)
		{
			return feat_dists_[idx]->getMean();
		};

		FeatureStatePtr drawFeatureSample(int idx, double timestamp)
		{
			return feat_dists_[idx]->drawSample(timestamp);
		}

		ObjectStatePtr drawObjectSample(int idx, double timestamp)
		{
			return targets_[idx]->drawSample(timestamp);
		}

		double getSampleProbability(ObjectStatePtr state, int idx, double timestamp)
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
		std::vector<MCMCSamplePtr> 		samples_;

		std::vector<TargetDistPtr> 		targets_;
		std::vector<FeatureDistPtr> 	feat_dists_;

		std::vector<double> 					obj_motion_params_;
		std::vector<double> 					feat_motion_params_;
		std::vector<double> 					cam_motion_params_;
	};

	typedef boost::shared_ptr<PosteriorDist> PosteriorDistPtr;
}; // NAMESPACE
#endif // _MCMC_SAMPLE_H_
