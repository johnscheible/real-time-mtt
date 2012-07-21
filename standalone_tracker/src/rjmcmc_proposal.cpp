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

#include <proposal/rjmcmc_proposal.h>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace people;

void RJMCMCProposal::setParameters(const std::string &name, const std::string &value)
{
	if (name == "ProbMoveAdd")	
		move_prob_[MoveAdd] = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveDelete")
		move_prob_[MoveDelete] = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveStay")
		move_prob_[MoveStay] = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveLeave")
		move_prob_[MoveLeave] = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveUpdate")
		move_prob_[MoveUpdate] = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveCamUpdate")
		move_prob_[MoveCamUpdate] = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveFeatAdd")
		assert(0);
	else if (name == "ProbMoveFeatDelete")
		assert(0);
	else if (name == "ProbMoveFeatStay")
		move_prob_[MoveFeatStay] = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveFeatLeave")
		move_prob_[MoveFeatLeave] = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveFeatUpdate")
		move_prob_[MoveFeatUpdate] = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveInteractionFlip")
		move_prob_[MoveInteractionFlip] = boost::lexical_cast<double>(value);
#ifdef VEL_STATE
	else if (name == "MotionSigmaX")
		motion_sigma_x_ = boost::lexical_cast<double>(value);
	else if (name == "MotionSigmaY")
		motion_sigma_y_ = boost::lexical_cast<double>(value);
	else if (name == "MotionSigmaZ")
		motion_sigma_z_ = boost::lexical_cast<double>(value);
	else if (name == "MotionSigmaVX")
		motion_sigma_vx_ = boost::lexical_cast<double>(value);
	//else if (name == "MotionSigmaVY")
	//	motion_sigma_vy_ = boost::lexical_cast<double>(value);
	else if (name == "MotionSigmaVZ")
		motion_sigma_vz_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbSigmaX")
		pert_sigma_x_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbSigmaY")
		pert_sigma_y_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbSigmaZ")
		pert_sigma_z_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbSigmaVX")
		pert_sigma_vx_ = boost::lexical_cast<double>(value);
	//else if (name == "PerturbSigmaVY")
	//	pert_sigma_vy_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbSigmaVZ")
		pert_sigma_vz_ = boost::lexical_cast<double>(value);
#else
	else if (name == "PerturbSigmaX")
		sigma_x_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbSigmaY")
		sigma_y_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbSigmaZ")
		sigma_z_ = boost::lexical_cast<double>(value);
#endif
	else if (name == "DetectionSigmaX")
		det_sigma_x_ = boost::lexical_cast<double>(value);
	else if (name == "DetectionSigmaY")
		det_sigma_y_ = boost::lexical_cast<double>(value);
	else if (name == "DetectionSigmaH")
		det_sigma_h_ = boost::lexical_cast<double>(value);
	else if (name == "ProbInteractionFlip")
		prob_interaction_flip_ = boost::lexical_cast<double>(value);
	// camera perturbation parameters
	else if (name == "PerturbCamSigmaX")
		pert_cam_sigma_x_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbCamSigmaZ")
		pert_cam_sigma_z_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbCamSigmaYaw")
		pert_cam_sigma_yaw_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbCamSigmaV")
		pert_cam_sigma_v_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbCamSigmaHorizon")
		pert_cam_sigma_horizon_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbFeatSigmaX")
		pert_feat_sigma_x_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbFeatSigmaZ")
		pert_feat_sigma_z_ = boost::lexical_cast<double>(value);
	else
		std::cout << "RJMCMCProposal : not defined parameter " << name << ":" << value << std::endl;
}

SampleInfo RJMCMCProposal::drawNewSample(MCMCSamplePtr sample)
{
	double rval = rng_.uniform((double)0.0, (double)1.0 - EPS);
	SampleInfo ret;

	if((rval -= move_prob_[MoveAdd]) < 0) {
		ret = addTarget(sample);
	}
	else if((rval -= move_prob_[MoveDelete]) < 0) {
		ret = deleteTarget(sample);
	}
	else if((rval -= move_prob_[MoveStay]) < 0) {
		ret = stayTarget(sample);
	}
	else if((rval -= move_prob_[MoveLeave]) < 0) {
		ret = leaveTarget(sample);
	}
	else if((rval -= move_prob_[MoveUpdate]) < 0){ // update
		ret = updateTarget(sample);
	}
	else if ((rval -= move_prob_[MoveCamUpdate]) < 0)  {
		ret = updateCamera(sample);
	}
	else if ((rval -= move_prob_[MoveFeatAdd]) < 0)  {
		// no add 
		assert(0);
	}
	else if ((rval -= move_prob_[MoveFeatDelete]) < 0)  {
		// no delete
		assert(0);
	}
	else if ((rval -= move_prob_[MoveFeatStay]) < 0)  {
		ret = stayFeature(sample);
	}
	else if ((rval -= move_prob_[MoveFeatLeave]) < 0)  {
		ret = leaveFeature(sample);
	}
	else if ((rval -= move_prob_[MoveFeatUpdate]) < 0)  {
		ret = updateFeature(sample);
	}
	else if ((rval -= move_prob_[MoveInteractionFlip]) < 0)  {
		ret = flipInteraction(sample);
	}
	else {
		assert(0);
	}

	return ret;
}

void RJMCMCProposal::initCache(MCMCSamplePtr sample)
{
	assert(prev_dist_.get() != NULL);
	// initialize cache
	set_add_.clear(); set_delete_.clear(); set_stay_.clear();	set_leave_.clear();
	set_feat_stay_.clear(); set_feat_leave_.clear();

	for(int i = 0; i < sample->getNumTargets(); i++)
	{
		if(sample->getExistance(i))
		{
			if(i < prev_dist_->getNumTargets()) // existing target
				set_leave_.push_back(i);
			else // new target
				set_delete_.push_back(i);
		}
		else
		{
			if(i < prev_dist_->getNumTargets()) // existing target
				set_stay_.push_back(i);
			else // new target
				set_add_.push_back(i);
		}
	}

	for(int i = 0; i < sample->getNumGFeats(); i++) 
	{
		if(i < prev_dist_->getNumFeats()) { // existed in previos frame
			if(sample->getFeatValidity(i)) {
				set_feat_leave_.push_back(i);
			}
			else {
				set_feat_stay_.push_back(i);
			}
		}
	}
	printCache();
}

void RJMCMCProposal::printCache()
{
	std::cout << "set_add : " ;
	for(size_t i = 0; i < set_add_.size(); i++)
		std::cout << set_add_[i] << ", ";
	std::cout << std::endl;
	std::cout << "set_delete : " ;
	for(size_t i = 0; i < set_delete_.size(); i++)
		std::cout << set_delete_[i] << ", ";
	std::cout << std::endl;
	std::cout << "set_stay : " ;
	for(size_t i = 0; i < set_stay_.size(); i++)
		std::cout << set_stay_[i] << ", ";
	std::cout << std::endl;
	std::cout << "set_leave : " ;
	for(size_t i = 0; i < set_leave_.size(); i++)
		std::cout << set_leave_[i] << ", ";
	std::cout << std::endl;
	std::cout << "set_feat_stay : " ;
	for(size_t i = 0; i < set_feat_stay_.size(); i++)
		std::cout << set_feat_stay_[i] << ", ";
	std::cout << std::endl;
	std::cout << "set_feat_leave : " ;
	for(size_t i = 0; i < set_feat_leave_.size(); i++)
		std::cout << set_feat_leave_[i] << ", ";
	std::cout << std::endl;
}

void RJMCMCProposal::updateCache(const SampleInfo &info, bool accepted)
{
	// update the cache
	std::vector<int>::iterator it;
	int i = info.idx_;

	if(accepted) {
		switch(info.type_)
		{
		case MoveAdd:
			// move the target from add cache to delete cache
			it = std::find(set_add_.begin(), set_add_.end(), i);
			assert(it != set_add_.end());
			set_add_.erase(it);
			set_delete_.push_back(i);
			break;
		case MoveDelete:
			it = std::find(set_delete_.begin(), set_delete_.end(), i);
			assert(it != set_delete_.end());
			set_delete_.erase(it);
			set_add_.push_back(i);
			break;
		case MoveStay:
			it = std::find(set_stay_.begin(), set_stay_.end(), i);
			assert(it != set_stay_.end());
			set_stay_.erase(it);
			set_leave_.push_back(i);
			break;
		case MoveLeave:
			it = std::find(set_leave_.begin(), set_leave_.end(), i);
			assert(it != set_leave_.end());
			set_leave_.erase(it);
			set_stay_.push_back(i);
			break;
		case MoveFeatStay:
			it = std::find(set_feat_stay_.begin(), set_feat_stay_.end(), i);
			assert(it != set_feat_stay_.end());
			set_feat_stay_.erase(it);
			set_feat_leave_.push_back(i);
			break;
		case MoveFeatLeave:
			it = std::find(set_feat_leave_.begin(), set_feat_leave_.end(), i);
			assert(it != set_feat_leave_.end());
			set_feat_leave_.erase(it);
			set_feat_stay_.push_back(i);
			break;
		case MoveUpdate:
		default:
			// nothing to be done.
			break;
		}
	}
}

void RJMCMCProposal::dbgCheckCache(MCMCSamplePtr sample)
{
// debug check whether the cache is correct
	assert(sample->getNumTargets() == (int)(set_add_.size() + set_delete_.size() + set_stay_.size() + set_leave_.size()));

	for(int i = 0; i < sample->getNumTargets(); i++)
	{
		if(sample->getExistance(i))
		{
			if(i < prev_dist_->getNumTargets()) // existing target
				assert(set_leave_.end() != std::find(set_leave_.begin(), set_leave_.end(), i));
			else
				assert(set_delete_.end() != std::find(set_delete_.begin(), set_delete_.end(), i));
		}
		else
		{
			if(i < prev_dist_->getNumTargets()) // existing target
				assert(set_stay_.end() != std::find(set_stay_.begin(), set_stay_.end(), i));
			else
				assert(set_add_.end() != std::find(set_add_.begin(), set_add_.end(), i));
		}
	}
}

SampleInfo	RJMCMCProposal::addTarget(MCMCSamplePtr sample)
{
	SampleInfo ret;
	if(set_add_.size() == 0) { // no candidate to be added
		ret.type_ = MoveNone;
		return ret;
	}
	ret.type_ = MoveAdd;

	unsigned int idx = floor(rng_.uniform((double)0.0, (double)set_add_.size() - EPS)); // randomly select one target
	my_assert(idx < set_add_.size());
	ret.idx_ = set_add_[idx];
#if 1
	cv::Rect rt = dets_[ret.idx_];
	rt.x += rng_.gaussian(det_sigma_x_ * rt.height);
	rt.y += rng_.gaussian(det_sigma_y_ * rt.height);
	rt.height += rng_.gaussian(det_sigma_h_ * rt.height);

	PeopleStatePtr state = sample->getCamState()->iproject(rt);
#else
	PeopleStatePtr state = sample->getCamState()->iproject(dets_[ret.idx_]);
	double temp;
	temp = state->getX() + rng_.gaussian(det_sigma_x_); state->setX(temp);
	temp = state->getY() + rng_.gaussian(det_sigma_y_); state->setY(temp);
	temp = state->getZ() + rng_.gaussian(det_sigma_z_); state->setZ(temp);
#endif
	// state->x_ += rng_.gaussian(det_sigma_x_);
	// state->y_ += rng_.gaussian(det_sigma_y_);
	// state->z_ += rng_.gaussian(det_sigma_z_);
#ifdef VEL_STATE
	// setting 0 value for velocty, sigma_vx_ will be dependent on the num of frames being tracked
	state->setVX(0.0); state->setVY(0.0); state->setVZ(0.0);
	// state->vx_ = state->vy_ = state->vz_ = 0.0;
#endif
	state->setTS(timestamp_);
	// state->timesec_ = timestamp_;
	ret.state_ = state;

	return ret;
}

SampleInfo	RJMCMCProposal::deleteTarget(MCMCSamplePtr sample)
{
	SampleInfo ret;
	if(set_delete_.size() == 0) { // no candidate to be deleted
		ret.type_ = MoveNone;
		return ret;
	}
	ret.type_ = MoveDelete;

	unsigned int idx = floor(rng_.uniform((double)0.0, (double)set_delete_.size() - EPS)); // randomly select one target
#ifdef DEBUG
	assert(idx < set_delete_.size());
#endif
	ret.idx_ = set_delete_[idx];
	return ret;
}

SampleInfo	RJMCMCProposal::stayTarget(MCMCSamplePtr sample)
{
	SampleInfo ret;
	if(set_stay_.size() == 0) { // no candidate to stay
		ret.type_ = MoveNone;
		return ret;
	}
	ret.type_ = MoveStay;

	unsigned int idx = floor(rng_.uniform((double)0.0, (double)set_stay_.size() - EPS)); // randomly select one target
#ifdef DEBUG
	assert(idx < set_stay_.size());
#endif
	ret.idx_ = set_stay_[idx];

	PeopleStatePtr state;
	if(dets_[ret.idx_].width != 0 && dets_[ret.idx_].height != 0 // if exist a detection, otherwise draw from prior distribution
		&& rng_.uniform((double)0.0f, (double)1.0f - EPS) < 0.5) {
		// state = mgr_->getPeopleStateFromRect(dets_[ret.idx_], sample->getCamState());
#if 1
		cv::Rect rt = dets_[ret.idx_];
		rt.x += rng_.gaussian(det_sigma_x_ * rt.height);
		rt.y += rng_.gaussian(det_sigma_y_ * rt.height);
		rt.height += rng_.gaussian(det_sigma_h_ * rt.height);

		state = sample->getCamState()->iproject(rt);
#else
		state = sample->getCamState()->iproject(dets_[ret.idx_]);

		double temp;
		temp = state->getX() + rng_.gaussian(det_sigma_x_); state->setX(temp);
		temp = state->getY() + rng_.gaussian(det_sigma_y_); state->setY(temp);
		temp = state->getZ() + rng_.gaussian(det_sigma_z_); state->setZ(temp);
#endif

#ifdef VEL_STATE
		TargetDistPtr target = prev_dist_->getTarget(ret.idx_);
		PeopleStatePtr prev_mean = target->getMean();

		state->setVX(prev_mean->getVX());
		state->setVY(0.0);
		state->setVZ(prev_mean->getVZ());
#endif
		state->setTS(timestamp_);
	}
	else {
#ifdef VEL_STATE
		TargetDistPtr target = prev_dist_->getTarget(ret.idx_);
		std::vector<PeopleStatePtr> samples = target->getStates();

		int idx = rng_.uniform((int)0, (int)samples.size());
		state = samples[idx]->clone();

		double dt = timestamp_ - state->getTS(); // uncertainty over time
		double vel_factor = get_vel_factor(target->getFrames());

		double temp;
		temp = state->getX() + rng_.gaussian(motion_sigma_x_ * dt); state->setX(temp);
		temp = state->getY() + rng_.gaussian(motion_sigma_y_ * dt); state->setY(temp);
		temp = state->getZ() + rng_.gaussian(motion_sigma_z_ * dt); state->setZ(temp);

		temp = state->getVX() + rng_.gaussian(motion_sigma_vx_ * dt * vel_factor); state->setVX(temp);
		state->setVY(0.0); 
		temp = state->getVZ() + rng_.gaussian(motion_sigma_vz_ * dt * vel_factor); state->setVZ(temp);
		
		assert(state->getTS() - timestamp_ < 0.001);
		state = state->predict(timestamp_);
#else
		state = prev_dist_->drawSample(ret.idx_, timestamp_);
#endif
	}
	ret.state_ = state;

	return ret;
}

double RJMCMCProposal::stateProposalStay(MCMCSamplePtr sample, PeopleStatePtr state, int idx)
{
	double det_proposal = 0.0;
	double prev_proposal = 0.0;
#ifdef VEL_STATE
	TargetDistPtr target = prev_dist_->getTarget(idx);
	std::vector<PeopleStatePtr> samples = target->getStates();
	
	double det_cov;
	cv::Mat cov = cv::Mat::eye(5, 5, CV_64F);
	cv::Mat A = cv::Mat::eye(5, 5, CV_64F);
	cv::Mat icov;
	double dt = timestamp_ - samples[0]->getTS(); // uncertainty over time
	double vel_factor = get_vel_factor(target->getFrames());
	
	A.at<double>(0, 3) = dt; A.at<double>(2, 4) = dt;

	cov.at<double>(0, 0) = pow(motion_sigma_x_ * dt, 2);
	cov.at<double>(1, 1) = pow(motion_sigma_y_ * dt, 2);
	cov.at<double>(2, 2) = pow(motion_sigma_z_ * dt, 2);
	cov.at<double>(3, 3) = pow(motion_sigma_vx_ * dt * vel_factor, 2);
	cov.at<double>(4, 4) = pow(motion_sigma_vz_ * dt * vel_factor, 2);
	// A * Sigma * A'
	icov = A * cov; icov = icov * A.t();
	
	det_cov = cv::determinant(icov);
	icov = icov.inv();
	for(size_t i = 0; i < samples.size(); i++) {
		cv::Mat x(5, 1, CV_64F);
		cv::Mat m(5, 1, CV_64F);
		m.at<double>(0, 0) = samples[i]->getX();	m.at<double>(1, 0) = samples[i]->getY();	m.at<double>(2, 0) = samples[i]->getZ();
		m.at<double>(3, 0) = samples[i]->getVX();	m.at<double>(4, 0) = samples[i]->getVZ();

		// predict
		m = A * m;
		x.at<double>(0, 0) = state->getX();		x.at<double>(1, 0) = state->getY();		x.at<double>(2, 0) = state->getZ();
		x.at<double>(3, 0) = state->getVX();	x.at<double>(4, 0) = state->getVZ();
		prev_proposal += exp(log_gaussian_prob(x, m, icov, det_cov));
	}
	prev_proposal /= (double)samples.size();
#else
	prev_proposal = prev_dist_->getSampleProbability(state, idx, timestamp_);
#endif

	if(dets_[idx].width != 0 && dets_[idx].height != 0) {
#if 1
		cv::Rect rt = sample->getCamState()->project(state);
		cv::Rect drt = dets_[idx];

		det_proposal = gaussian_prob(rt.x, drt.x, det_sigma_x_ * drt.height);
		det_proposal *= gaussian_prob(rt.y, drt.y, det_sigma_y_ * drt.height);
		det_proposal *= gaussian_prob(rt.height, drt.height, det_sigma_h_ * drt.height);
#else
		PeopleStatePtr dstate = sample->getCamState()->iproject(dets_[idx]);
		det_proposal = gaussian_prob(state->getX(), dstate->getX(), det_sigma_x_)
								* gaussian_prob(state->getY(), dstate->getY(), det_sigma_y_)
								* gaussian_prob(state->getZ(), dstate->getZ(), det_sigma_z_);
#endif
	}
	else { // if no detection
		det_proposal = prev_proposal;
	}
	my_assert(det_proposal + prev_proposal != 0.0f);
	// my_assert(prev_proposal != 0.0f);
	my_assert(!isnan(det_proposal));
	my_assert(!isnan(prev_proposal));
	return (det_proposal + prev_proposal) / 2;
}

SampleInfo	RJMCMCProposal::leaveTarget(MCMCSamplePtr sample)
{
	SampleInfo ret;
	if(set_leave_.size() == 0) { // no candidate to leave
		ret.type_ = MoveNone;
		return ret;
	}
	ret.type_ = MoveLeave;

	unsigned int idx = floor(rng_.uniform((double)0.0, (double)set_leave_.size() - EPS)); // randomly select one target
#ifdef DEBUG
	assert(idx < set_leave_.size());
#endif
	ret.idx_ = set_leave_[idx];

	return ret;
}

SampleInfo RJMCMCProposal::updateTarget(MCMCSamplePtr sample)
{
	SampleInfo ret;
	if(set_delete_.size() + set_leave_.size() == 0) { // no candidate to be updated
		ret.type_ = MoveNone;
		return ret;
	}
	ret.type_ = MoveUpdate;

	int idx = floor(rng_.uniform((double)0.0, (double)set_leave_.size() + set_delete_.size() - EPS)); 
	if(idx < (int)set_leave_.size()) { // from existing targets - stayed
		idx = set_leave_[idx];
	}
	else
	{ // new target - added
		idx -= set_leave_.size();
		idx = set_delete_[idx];
	}
	ret.idx_ = idx;

	PeopleStatePtr state;
#ifdef VEL_STATE
	state = sample->getState(ret.idx_)->clone();

	if(ret.idx_ < (unsigned int)prev_dist_->getNumTargets()) { 
		// perturb velocity
		double dt = timestamp_ - prev_dist_->getTimeStamp();
		TargetDistPtr target = prev_dist_->getTarget(ret.idx_);
		int frames = target->getFrames();

		double vel_factor = get_vel_factor(frames);
		double dx = rng_.gaussian(pert_sigma_x_ * dt); 
		double dy = rng_.gaussian(pert_sigma_y_ * dt); 
		double dz = rng_.gaussian(pert_sigma_z_ * dt);
		double dvx = rng_.gaussian(pert_sigma_vx_ * dt * vel_factor ); 
		double dvz = rng_.gaussian(pert_sigma_vz_ * dt * vel_factor );
		// z += A * dz : equal to N(0, ASA')

		double temp;
		temp = state->getX() + dx + dvx * dt; state->setX(temp);
		temp = state->getY() + dy; 			  state->setY(temp);
		temp = state->getZ() + dz + dvz * dt; state->setZ(temp);
		temp = state->getVX() + dvx; state->setVX(temp);
		temp = state->getVZ() + dvz; state->setVZ(temp);

		state->setTS(timestamp_);
	}
	else { // newly entered, no previous time information
		double temp;
#if 1
		temp = state->getX() + rng_.gaussian(0.025); state->setX(temp);
		temp = state->getY() + rng_.gaussian(0.05); state->setY(temp);
		temp = state->getZ() + rng_.gaussian(0.05); state->setZ(temp);
#else
		temp = state->getX() + rng_.gaussian(pert_sigma_x_ * dt); state->setX(temp);
		temp = state->getY() + rng_.gaussian(pert_sigma_y_ * dt); state->setY(temp);
		temp = state->getZ() + rng_.gaussian(pert_sigma_z_ * dt); state->setZ(temp);
#endif
		// state->x_ += rng_.gaussian(det_sigma_x_ / 2);
		// state->y_ += rng_.gaussian(det_sigma_y_ / 2);
		// state->z_ += rng_.gaussian(det_sigma_z_ / 2);
		state->setTS(timestamp_);
	}
#else
	state = sample->getState(ret.idx_)->clone();

	double temp;
	temp = state->getX() + rng_.gaussian(sigma_x_); state->setX(temp);
	temp = state->getY() + rng_.gaussian(sigma_y_);	state->setY(temp);
	temp = state->getZ() + rng_.gaussian(sigma_z_); state->setZ(temp);
	state->setTS(timestamp_);
#endif
	ret.state_ = state;

	return ret;
}

SampleInfo RJMCMCProposal::updateCamera(MCMCSamplePtr sample)
{
	SampleInfo ret;
	ret.type_ = MoveCamUpdate;
	ret.cam_state_ = sample->getCamState()->clone();

	double dt = timestamp_ - prev_dist_->getTimeStamp();
	// see updateTarget for reference
	double dx, dz;
	dx = -ret.cam_state_->getV() * sin(ret.cam_state_->getYaw()) * dt;
	dz = ret.cam_state_->getV() * cos(ret.cam_state_->getYaw()) * dt;

	double temp;
	temp = ret.cam_state_->getYaw() + rng_.gaussian(pert_cam_sigma_yaw_ * dt); ret.cam_state_->setYaw(temp);
	temp = ret.cam_state_->getV() + rng_.gaussian(pert_cam_sigma_v_ * dt); ret.cam_state_->setV(temp);
	temp = ret.cam_state_->getHorizon() + rng_.gaussian(pert_cam_sigma_horizon_ * dt); ret.cam_state_->setHorizon(temp);
	// dt * vx, dt * vz

	// add location perturbation + go back to previous time location
	temp = ret.cam_state_->getX() + rng_.gaussian(pert_cam_sigma_x_ * dt) - dx; ret.cam_state_->setX(temp);
	temp = ret.cam_state_->getZ() + rng_.gaussian(pert_cam_sigma_z_ * dt) - dz; ret.cam_state_->setZ(temp);
	ret.cam_state_->setTS(prev_dist_->getTimeStamp());

	ret.cam_state_ = ret.cam_state_->predict(timestamp_);

	return ret;
}

SampleInfo RJMCMCProposal::stayFeature(MCMCSamplePtr sample)
{
	SampleInfo ret;
	if(set_feat_stay_.size() == 0) {
		ret.type_ = MoveNone;
		return ret;
	}
	ret.type_ = MoveFeatStay;
	unsigned int idx = floor(rng_.uniform((double)0.0, (double)set_feat_stay_.size() - EPS)); // randomly select one target
	ret.idx_ = set_feat_stay_[idx];
	// choose one from previous samples
	ret.feat_state_ = prev_dist_->drawFeatSample(ret.idx_, timestamp_);
	assert(ret.feat_state_.get());
	return ret;
}

SampleInfo RJMCMCProposal::leaveFeature(MCMCSamplePtr sample)
{
	SampleInfo ret;
	if(set_feat_leave_.size() == 0) {
		ret.type_ = MoveNone;
		return ret;
	}
	ret.type_ = MoveFeatLeave;
	unsigned int idx = floor(rng_.uniform((double)0.0, (double)set_feat_leave_.size() - EPS)); // randomly select one target
	ret.idx_ = set_feat_leave_[idx];
	return ret;
}

SampleInfo RJMCMCProposal::updateFeature(MCMCSamplePtr sample)
{
	SampleInfo ret;
	if(sample->getNumGFeats() == 0) {
		ret.type_ = MoveNone;
		return ret;
	}
	ret.type_ = MoveFeatUpdate;

	int nfeats = sample->getNumGFeats() - prev_dist_->getNumFeats() + set_feat_leave_.size();
	unsigned int idx = floor(rng_.uniform((double)0.0, (double)nfeats - EPS));
	// if existing just sample from the prior dist - cause it's a dirac delta
	// if not, sample from gaussian noise
	GFeatStatePtr feat_state;
	if(idx < set_feat_leave_.size()) {
		ret.idx_ = set_feat_leave_[idx];
		feat_state = prev_dist_->drawFeatSample(ret.idx_, timestamp_);
	}
	else {
		ret.idx_ = idx - set_feat_leave_.size() + prev_dist_->getNumFeats();

		if(!sample->getFeatValidity(ret.idx_) || ret.idx_ >= sample->getNumGFeats()) { 
			std::cout << "Not valid why ? => " << ret.idx_ << std::endl;
			ret.type_ = MoveNone;
			return ret;
		}

		// perturb 
		feat_state = sample->getFeatState(ret.idx_)->clone();

		double temp;
		temp = feat_state->getX() + rng_.gaussian(pert_feat_sigma_x_); feat_state->setX(temp);
		feat_state->setY(0.0);
		temp = feat_state->getZ() + rng_.gaussian(pert_feat_sigma_z_); feat_state->setZ(temp);

		// feat_state->x_ += rng_.gaussian(pert_feat_sigma_x_);
		// feat_state->y_ += rng_.gaussian(pert_feat_sigma_y_);
		// feat_state->z_ += rng_.gaussian(pert_feat_sigma_z_);
	}

	if(!sample->getFeatValidity(ret.idx_)) { 
		std::cout << "Not valid why ? => " << ret.idx_ << std::endl;
	}

	my_assert(sample->getFeatValidity(ret.idx_));
	ret.feat_state_ = feat_state;

	return ret;
}

SampleInfo RJMCMCProposal::flipInteraction(MCMCSamplePtr sample)
{
#ifndef VEL_STATE
	{
		SampleInfo ret;
		ret.type_ = MoveNone;
		return ret;
	}
#endif
#if 1
	SampleInfo ret;
	if((set_leave_.size()) <= 1) { // no candidate to be updated
		ret.type_ = MoveNone;
		return ret;
	}
	ret.type_ = MoveInteractionFlip;

	unsigned int num_targets = set_leave_.size();
	unsigned int num_pairs; 
	getPairIndex(0, num_targets, num_pairs);

	unsigned int idx = floor(rng_.uniform((double)0.0, (double)num_pairs - EPS));
	getPairFromIndex(ret.idx_, ret.idx2_, num_targets, idx);

	ret.idx_ = set_leave_[ret.idx_];
	ret.idx2_ = set_leave_[ret.idx2_];

	// check the life time of each track... //////////////////////
	TargetDistPtr t1 = prev_dist_->getTarget(ret.idx_);
	TargetDistPtr t2 = prev_dist_->getTarget(ret.idx2_);

	if(t1->getFrames() <= min_frames_group_ || t2->getFrames() <= min_frames_group_) {
		ret.type_ = MoveNone;
		return ret;
	}
	//////////////////////////////////////////////////////////////
#else
	SampleInfo ret;
	if((set_delete_.size() + set_leave_.size()) <= 1) { // no candidate to be updated
		ret.type_ = MoveNone;
		return ret;
	}
	ret.type_ = MoveInteractionFlip;
	
	unsigned int num_targets = set_leave_.size() + set_delete_.size();
	unsigned int num_pairs; getPairIndex(0, num_targets, num_pairs);

	unsigned int idx = floor(rng_.uniform((double)0.0, (double)num_pairs - EPS));
	getPairFromIndex(ret.idx_, ret.idx2_, num_targets, idx);

	if(ret.idx_ < set_leave_.size()) { // from existing targets - stayed
		ret.idx_ = set_leave_[ret.idx_];
	}
	else
	{ // new target - added
		ret.idx_ -= set_leave_.size();
		ret.idx_ = set_delete_[ret.idx_];
	}

	if(ret.idx2_ < set_leave_.size()) { // from existing targets - stayed
		ret.idx2_ = set_leave_[ret.idx2_];
	}
	else
	{ // new target - added
		ret.idx2_ -= set_leave_.size();
		ret.idx2_ = set_delete_[ret.idx2_];
	}
#endif
	unsigned int temp = min(ret.idx_, ret.idx2_);
	ret.idx2_ = max(ret.idx_, ret.idx2_);
	ret.idx_ = temp;

	if(floor(rng_.uniform(0.0f, 1.0f) < prob_interaction_flip_))
		ret.group_mode_ = !sample->getInteractionMode(ret.idx_, ret.idx2_);
	else
		ret.group_mode_ = sample->getInteractionMode(ret.idx_, ret.idx2_);

	return ret;
}

// likelihood computation.
double RJMCMCProposal::getProposalLogLkhood(const MCMCSamplePtr sample, const SampleInfo &info)
{
	double ret = 0;
	switch(info.type_)
	{
	case MoveAdd:
		ret += log(move_prob_[MoveDelete]);
		ret += log(set_add_.size());
		ret -= log(move_prob_[MoveAdd]);
		ret -= log(set_delete_.size() + 1);
		break;
	case MoveDelete:
		ret += log(move_prob_[MoveAdd]);
		ret += log(set_delete_.size());
		ret -= log(move_prob_[MoveDelete]);
		ret -= log(set_add_.size() + 1);
		break;
	case MoveStay:
		ret += log(move_prob_[MoveLeave]);
		ret += log(set_stay_.size());
		ret -= log(move_prob_[MoveStay]);
		ret -= log(set_leave_.size() + 1);
		// need to compensate the location proposal
		// ret -= log(stateProposalStay(sample, info.state_, info.idx_));
		break;
	case MoveLeave:
		ret += log(move_prob_[MoveStay]);
		ret += log(set_leave_.size());
		ret -= log(move_prob_[MoveLeave]);
		ret -= log(set_stay_.size() + 1);
		// need to compensate the location proposal
		// ret += log(stateProposalStay(sample, sample->getState(info.idx_), info.idx_));
		break;
	case MoveFeatAdd:
		assert(0);
		break;
	case MoveFeatDelete:
		assert(0);
		break;
	case MoveFeatStay:
		ret += log(move_prob_[MoveFeatLeave]);
		ret += log(set_feat_stay_.size());
		ret -= log(move_prob_[MoveFeatStay]);
		ret -= log(set_feat_leave_.size() + 1);
		// need to compensate location proposals -> 1/N
		// ret -= log(prev_dist_->getFeatureDist(info.idx_)->getSampleProbability(info.feat_state_, timestamp_));
		break;
	case MoveFeatLeave:
		ret += log(move_prob_[MoveFeatStay]);
		ret += log(set_feat_leave_.size());
		ret -= log(move_prob_[MoveFeatLeave]);
		ret -= log(set_feat_stay_.size() + 1);
		// need to compensate location proposals -> 1/N
		// ret += log(prev_dist_->getFeatureDist(info.idx_)->getSampleProbability(info.feat_state_, timestamp_));
		break;
	case MoveFeatUpdate:
	case MoveInteractionFlip:
	case MoveUpdate:
	case MoveCamUpdate:
		// nothing to do, it's balanced...
		ret = 0;
		break;
	default:
		assert(0);
		break;
	}
	return ret;
}
