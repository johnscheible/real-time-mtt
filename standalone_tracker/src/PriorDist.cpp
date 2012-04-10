#include <prior/PriorDist.h>
#include <boost/lexical_cast.hpp>
#include <iomanip>
#include <common/util.h>

using namespace people;

#ifdef VEL_STATE
void PriorDist::initMotionParameters()
{
	size_t nTargets = prev_dist_->getNumTargets();
	double dt = timestamp_ - prev_dist_->getTimeStamp();
	cv::Mat A = cv::Mat::eye(5, 5, CV_64F);
	cv::Mat Sigma = cv::Mat::eye(5, 5, CV_64F);
	cv::Mat temp;

	Sigma.at<double>(0, 0) = pow(motion_sigma_x_ * dt, 2);
	Sigma.at<double>(1, 1) = pow(motion_sigma_y_ * dt, 2);
	Sigma.at<double>(2, 2) = pow(motion_sigma_z_ * dt, 2);
	A.at<double>(0, 3) = dt; A.at<double>(2, 4) = dt;

	motion_sigma_invs_.clear();
	motion_sigma_dets_.clear();

	for(size_t i = 0; i < nTargets; i++) {
		TargetDistPtr target = prev_dist_->getTarget(i);
		int frames = target->getFrames();
		double vel_factor = get_vel_factor(frames);

		// more variance for less tracked targets
		Sigma.at<double>(3, 3) = pow(motion_sigma_vx_ * dt * vel_factor, 2);
		Sigma.at<double>(4, 4) = pow(motion_sigma_vz_ * dt * vel_factor, 2);

		// A * Sigma * A'
		temp = A * Sigma;
		temp = temp * A.t();
		
		motion_sigma_invs_.push_back(temp.inv());
		motion_sigma_dets_.push_back(cv::determinant(temp));
#if 0 // def MYDEBUG
		char msg[1000];

		for(int j = 0; j < (int)prev_dist_->getNumSamples(); j++) {
			sprintf(msg, "target %d %dth state\n [", (int)i, j);
			print_dbg_file(msg);
			
			MCMCSamplePtr sample = prev_dist_->getSample(j);
			PeopleStatePtr state = sample->getState(i);
			if(state.get() == NULL)
				continue;

			cv::Mat sm = state->getMatrix();
			for(int j = 0; j < 6; j++) {
				sprintf(msg, "%.5lf\t", sm.at<double>(j, 0));
				print_dbg_file(msg);
			}

			sprintf(msg, "]\n");
			print_dbg_file(msg);
		}

		sprintf(msg, "target %d covariance!\n", (int)i);
		print_dbg_file(msg);
		for(int j = 0; j < 6; j++) {
			for(int k = 0; k < 6; k++) {
				sprintf(msg, "%.8lf\t", temp.at<double>(j, k));
				print_dbg_file(msg);
			}
			sprintf(msg, "\n");
			print_dbg_file(msg);
		}

		sprintf(msg, "target %d det(cov) : %lf!\n", (int)i, cv::determinant(temp));
		print_dbg_file(msg);
#endif
#if 0
		std::cout << "target " << i << " covariance!" << std::endl;
		for(int j = 0; j < 6; j++) {
			for(int k = 0; k < 6; k++) {
				std::cout << setprecision(5) << temp.at<double>(j, k) << "\t";
			}
			std::cout << std::endl;
		}
		assert(0);
#endif
	}
}
#endif

void PriorDist::setParameters(const std::string &name, const std::string &value)
{
#ifdef VEL_STATE
	if (name == "MotionSigmaVX")
		motion_sigma_vx_ = boost::lexical_cast<double>(value);
	else if (name == "MotionSigmaVY")
		motion_sigma_vy_ = boost::lexical_cast<double>(value);
	else if (name == "MotionSigmaVZ")
		motion_sigma_vz_ = boost::lexical_cast<double>(value);
	else if (name == "MotionSigmaX")
		motion_sigma_x_ = boost::lexical_cast<double>(value);
	else if (name == "MotionSigmaY")
		motion_sigma_y_ = boost::lexical_cast<double>(value);
	else if (name == "MotionSigmaZ")
		motion_sigma_z_ = boost::lexical_cast<double>(value);
#else
	if (name == "MotionSigmaX")
		motion_sigma_x_ = boost::lexical_cast<double>(value);
	else if (name == "MotionSigmaY")
		motion_sigma_y_ = boost::lexical_cast<double>(value);
	else if (name == "MotionSigmaZ")
		motion_sigma_z_ = boost::lexical_cast<double>(value);
#endif
	else if (name == "ProbStay")
		prob_stay_ = boost::lexical_cast<double>(value);
	else if (name == "ProbEnter")
		prob_enter_ = boost::lexical_cast<double>(value);
	else
		std::cout << "PriorDist : not defined parameter " << name << ":" << value << std::endl;
}

void PriorDist::initCache(MCMCSamplePtr sample)
{
	motion_cache_ = cv::Mat(prev_dist_->getNumSamples(), sample->getNumTargets(), CV_64F);
	// initialize cache
	for(int i = 0; i < sample->getNumTargets(); i++)
	{
		computeOneTargetMotionPrior(sample, i);
		for(int j = 0; j < one_target_motion_cache_.rows; j++)  {
			motion_cache_.at<double>(j, i) = one_target_motion_cache_.at<double>(j, 0);
		}
	}
}

void PriorDist::updateCache(const SampleInfo &info, bool accepted)
{
	if(accepted) {
		for(int j = 0; j < one_target_motion_cache_.rows; j++)  {
			motion_cache_.at<double>(j, info.idx_) = one_target_motion_cache_.at<double>(j, 0);
		}
	}
}

void PriorDist::computeNewSampletMotionPrior(const SampleInfo &info)
{
	int num_samples = prev_dist_->getNumSamples();
	one_target_motion_cache_ = cv::Mat(num_samples, 1, CV_64F);
	one_target_motion_cache_ = cv::Scalar(0.0);
	// std::cout << "timestamp ? " << timestamp_ - floor(timestamp_) << std::endl;
	// std::cout << "motion_prob : " ;
	if(info.idx_ < (unsigned int)prev_dist_->getNumTargets()) {
		// existing target
		if((info.type_ == MoveAdd) || (info.type_ == MoveStay) || (info.type_ == MoveUpdate)) {
			// existing now
			PeopleStatePtr state = info.state_;
			for(int i = 0; i < num_samples; i++) {
				MCMCSamplePtr prev_sample = prev_dist_->getSample(i);

				if(prev_sample->getExistance(info.idx_)) {
					PeopleStatePtr prev_state = prev_sample->getState(info.idx_);
					// std::cout << "dt " << dt << "sigma_x " << motion_sigma_x_ << std::endl;
					// stay
#ifdef VEL_STATE
					// didn't implemented
					assert(0);
#else
					double dt = timestamp_ - prev_state->timesec_;
					one_target_motion_cache_.at<double>(i, 0) = prob_stay_ // stay prob
					 	* gaussian_prob(state->x_, prev_state->x_, motion_sigma_x_ * dt) // location prob
					 	* gaussian_prob(state->y_, prev_state->y_, motion_sigma_y_ * dt)
					 	* gaussian_prob(state->z_, prev_state->z_, motion_sigma_z_ * dt);
					//std::cout << one_target_motion_cache_.at<double>(i, 0) << ", ";
#endif
				}
				else {
					// new enter
					one_target_motion_cache_.at<double>(i, 0) = prob_enter_;
				}
			}
		}
		else {
			// not existing now
			for(int i = 0; i < num_samples; i++) {
				MCMCSamplePtr prev_sample = prev_dist_->getSample(i);
				if(prev_sample->getExistance(info.idx_)) {
					// left
					one_target_motion_cache_.at<double>(i, 0) = 1 - prob_stay_;
				}
				else {
					// no enter
					one_target_motion_cache_.at<double>(i, 0) = 1 - prob_enter_;
				}
			}
		}
	}
	else {
		// new target
		for(int i = 0; i < num_samples; i++) {
			if((info.type_ == MoveAdd) || (info.type_ == MoveStay) || (info.type_ == MoveUpdate)) {
				// new enter
				one_target_motion_cache_.at<double>(i, 0) = prob_enter_;
			}
			else {
				// no enter
				one_target_motion_cache_.at<double>(i, 0) = 1 - prob_enter_;
			}
		}
	}
	// std::cout << std::endl;
}

void PriorDist::computeOneTargetMotionPrior(MCMCSamplePtr sample, int tid)
{
	int num_samples = prev_dist_->getNumSamples();
	one_target_motion_cache_ = cv::Mat(num_samples, 1, CV_64F);
	one_target_motion_cache_ = cv::Scalar(0.0);
	
	// std::cout << "motion_prob : " ;
	if(tid < prev_dist_->getNumTargets()) {
		// existing target
		if(sample->getExistance(tid)) {
			// existing now
			PeopleStatePtr state = sample->getState(tid);

			for(int i = 0; i < num_samples; i++) {
				MCMCSamplePtr prev_sample = prev_dist_->getSample(i);

				if(prev_sample->getExistance(tid)) {
					PeopleStatePtr prev_state = prev_sample->getState(tid);
					// std::cout << "dt " << dt << "sigma_x " << motion_sigma_x_ << std::endl;
					// stay
#ifdef VEL_STATE
					assert(0);
#else
					double dt = timestamp_ - prev_state->timesec_;
					one_target_motion_cache_.at<double>(i, 0) = prob_stay_ // stay prob
					 	* gaussian_prob(state->x_, prev_state->x_, motion_sigma_x_ * dt) // location prob
					 	* gaussian_prob(state->y_, prev_state->y_, motion_sigma_y_ * dt)
					 	* gaussian_prob(state->z_, prev_state->z_, motion_sigma_z_ * dt);
#endif
					// std::cout << one_target_motion_cache_.at<double>(i, 0) << ", ";
				}
				else {
					// new enter
					one_target_motion_cache_.at<double>(i, 0) = prob_enter_;
				}
			}
		}
		else {
			// not existing now
			for(int i = 0; i < num_samples; i++) {
				MCMCSamplePtr prev_sample = prev_dist_->getSample(i);
				if(prev_sample->getExistance(tid)) {
					// left
					one_target_motion_cache_.at<double>(i, 0) = 1 - prob_stay_;
				}
				else {
					// no enter
					one_target_motion_cache_.at<double>(i, 0) = 1 - prob_enter_;
				}
			}
		}
	}
	else {
		// new target
		for(int i = 0; i < num_samples; i++) {
			if(sample->getExistance(tid)) {
				// new enter
				one_target_motion_cache_.at<double>(i, 0) = prob_enter_;
			}
			else {
				// no enter
				one_target_motion_cache_.at<double>(i, 0) = 1 - prob_enter_;
			}
		}
	}
	// std::cout << std::endl;
}

double PriorDist::computeLogMotionPrior(const SampleInfo &info)
{
	double num = 0.0f, denom = 0.0f;
	// new samples
	for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
		double one_sample_prob = 1.0f;
		for(int j = 0; j < motion_cache_.cols; j++) {
			if( j == (int)info.idx_) {
				one_sample_prob *= one_target_motion_cache_.at<double>(i, 0);
			}
			else {
				one_sample_prob *= motion_cache_.at<double>(i, j);
			}
		}
		num += one_sample_prob;
	}
	// old samples
	for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
		double one_sample_prob = 1.0f;
		for(int j = 0; j < motion_cache_.cols; j++) {
			one_sample_prob *= motion_cache_.at<double>(i, j);
		}
		denom += one_sample_prob;
	}
	// std::cout << "motion numerator : " << std::setprecision(5) << num << " denom : " << denom << std::endl;
	
	num += EPS;
	denom += EPS;

#ifdef _DEBUG
	my_assert(num != 0.0f); my_assert(!isnan(num));
	my_assert(denom != 0.0f); my_assert(!isnan(denom));
#endif
	// normalize not necessary
	return log(num / denom);
}

//////////////////////////////////////////////////////////////////////////////////////
/////// Numerically stable version..
/////////////////////////////////////////////////////////////////////////////////////

// necessary to balance the motion prior
// without this, the location prior won't have any effect when it's farther than 1 sigma from the mean
#ifdef VEL_STATE
#define PROB_ENTER_CONST	-2000.0f
#else
#define PROB_ENTER_CONST	-50.0f
#endif
void PriorDistNS::computeNewSampletMotionPrior(const SampleInfo &info)
{
	int num_samples = prev_dist_->getNumSamples();
	one_target_motion_cache_ = cv::Mat(num_samples, 1, CV_64F);
	one_target_motion_cache_ = cv::Scalar(0.0);
	// std::cout << "timestamp ? " << timestamp_ - floor(timestamp_) << std::endl;
	// std::cout << "motion_prob : " ;
	if(info.idx_ < (unsigned int)prev_dist_->getNumTargets()) {
		// existing target
		if((info.type_ == MoveAdd) || (info.type_ == MoveStay) || (info.type_ == MoveUpdate)) {
			// existing now
			PeopleStatePtr state = info.state_;
			for(int i = 0; i < num_samples; i++) {
				MCMCSamplePtr prev_sample = prev_dist_->getSample(i);

				if(prev_sample->getExistance(info.idx_)) {
					PeopleStatePtr prev_state = prev_sample->getState(info.idx_);
#ifdef VEL_STATE
					cv::Mat x(5, 1, CV_64F);
					cv::Mat m(5, 1, CV_64F);
					// need to compare with current time prediction
					double dt = timestamp_ - prev_state->getTS(); 
					m.at<double>(0, 0) = prev_state->getX() + prev_state->getVX() * dt;	
					m.at<double>(1, 0) = prev_state->getY() + prev_state->getVY() * dt;	
					m.at<double>(2, 0) = prev_state->getZ() + prev_state->getVZ() * dt;
					m.at<double>(3, 0) = prev_state->getVX();	
					m.at<double>(4, 0) = prev_state->getVZ();

					x.at<double>(0, 0) = state->getX();		
					x.at<double>(1, 0) = state->getY();
					x.at<double>(2, 0) = state->getZ();
					x.at<double>(3, 0) = state->getVX();	
					x.at<double>(4, 0) = state->getVZ();

					one_target_motion_cache_.at<double>(i, 0) = 
						log(prob_stay_) // stay prob
					 	+ log_gaussian_prob(x, m, motion_sigma_invs_[info.idx_], motion_sigma_dets_[info.idx_]);
#else
					double dt = timestamp_ - prev_state->timesec_; 
					one_target_motion_cache_.at<double>(i, 0) = 
						log(prob_stay_) // stay prob
					 	+ log_gaussian_prob(state->getX(), prev_state->getX(), motion_sigma_x_ * dt) // location prob
					 	+ log_gaussian_prob(state->getY(), prev_state->getY(), motion_sigma_y_ * dt)
					 	+ log_gaussian_prob(state->getZ(), prev_state->getZ(), motion_sigma_z_ * dt);
#endif
				}
				else {
					// new enter
					one_target_motion_cache_.at<double>(i, 0) = log(prob_enter_) + PROB_ENTER_CONST;
				}
			}
		}
		else {
			// not existing now
			for(int i = 0; i < num_samples; i++) {
				MCMCSamplePtr prev_sample = prev_dist_->getSample(i);
				if(prev_sample->getExistance(info.idx_)) {
					// left
					one_target_motion_cache_.at<double>(i, 0) = log(1 - prob_stay_);
				}
				else {
					// no enter
					one_target_motion_cache_.at<double>(i, 0) = log(1 - prob_enter_) + PROB_ENTER_CONST;
				}
			}
		}
	}
	else {
		// new target
		for(int i = 0; i < num_samples; i++) {
			if((info.type_ == MoveAdd) || (info.type_ == MoveStay) || (info.type_ == MoveUpdate)) {
				// new enter
				one_target_motion_cache_.at<double>(i, 0) = log(prob_enter_);
			}
			else {
				// no enter
				one_target_motion_cache_.at<double>(i, 0) = log(1 - prob_enter_);
			}
		}
	}
	// std::cout << std::endl;
}

void PriorDistNS::computeOneTargetMotionPrior(MCMCSamplePtr sample, int tid)
{
	int num_samples = prev_dist_->getNumSamples();
	one_target_motion_cache_ = cv::Mat(num_samples, 1, CV_64F);
	one_target_motion_cache_ = cv::Scalar(0.0);
	
	// std::cout << "motion_prob : " ;
	if(tid < prev_dist_->getNumTargets()) {
		// existing target
		if(sample->getExistance(tid)) {
			// existing now
			PeopleStatePtr state = sample->getState(tid);

			for(int i = 0; i < num_samples; i++) {
				MCMCSamplePtr prev_sample = prev_dist_->getSample(i);

				if(prev_sample->getExistance(tid)) {
					PeopleStatePtr prev_state = prev_sample->getState(tid);
#ifdef VEL_STATE
					cv::Mat x(5, 1, CV_64F);
					cv::Mat m(5, 1, CV_64F);
					
					double dt = timestamp_ - prev_state->getTS(); 
					m.at<double>(0, 0) = prev_state->getX() + prev_state->getVX() * dt;	
					m.at<double>(1, 0) = prev_state->getY() + prev_state->getVY() * dt;	
					m.at<double>(2, 0) = prev_state->getZ() + prev_state->getVZ() * dt;
					m.at<double>(3, 0) = prev_state->getVX();	
					m.at<double>(4, 0) = prev_state->getVZ();	

					x.at<double>(0, 0) = state->getX();		
					x.at<double>(1, 0) = state->getY();
					x.at<double>(2, 0) = state->getZ();
					x.at<double>(3, 0) = state->getVX();	
					x.at<double>(4, 0) = state->getVZ();

					one_target_motion_cache_.at<double>(i, 0) = 
						log(prob_stay_) // stay prob
					 	+ log_gaussian_prob(x, m, motion_sigma_invs_[tid], motion_sigma_dets_[tid]);
#else
					double dt = timestamp_ - prev_state->timesec_;
					one_target_motion_cache_.at<double>(i, 0) = 
						log(prob_stay_) // stay prob
					 	+ log_gaussian_prob(state->getX(), prev_state->getX(), motion_sigma_x_ * dt) // location prob
					 	+ log_gaussian_prob(state->getY(), prev_state->getY(), motion_sigma_y_ * dt)
					 	+ log_gaussian_prob(state->getZ(), prev_state->getZ(), motion_sigma_z_ * dt);
#endif
				}
				else {
					// new enter
					one_target_motion_cache_.at<double>(i, 0) = log(prob_enter_) + PROB_ENTER_CONST;
				}
			}
		}
		else {
			// not existing now
			for(int i = 0; i < num_samples; i++) {
				MCMCSamplePtr prev_sample = prev_dist_->getSample(i);
				if(prev_sample->getExistance(tid)) {
					// left
					one_target_motion_cache_.at<double>(i, 0) = log(1 - prob_stay_);
				}
				else {
					// no enter
					one_target_motion_cache_.at<double>(i, 0) = log(1 - prob_enter_) + PROB_ENTER_CONST;
				}
			}
		}
	}
	else {
		// new target
		for(int i = 0; i < num_samples; i++) {
			if(sample->getExistance(tid)) {
				// new enter
				one_target_motion_cache_.at<double>(i, 0) = log(prob_enter_);
			}
			else {
				// no enter
				one_target_motion_cache_.at<double>(i, 0) = log(1 - prob_enter_);
			}
		}
	}
	// std::cout << std::endl;
}

double PriorDistNS::computeLogMotionPrior(const SampleInfo &info)
{
	double num = 0.0f, denom = 0.0f;
	double temp1[100], temp2[100];
	double max_log_prob = -DBL_MAX;

	my_assert(prev_dist_->getNumSamples() <= 100);

	if(prev_dist_type_ == "independent") {
		for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
			temp1[i] = one_target_motion_cache_.at<double>(i, 0);
			temp2[i] = motion_cache_.at<double>(i, (int)info.idx_);

			max_log_prob = max(max_log_prob, temp1[i]);
			max_log_prob = max(max_log_prob, temp2[i]);
		}
	}
	else {
		// dependent prior
		// new samples
		for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
			double one_sample_prob = 0.0f;
			for(int j = 0; j < motion_cache_.cols; j++) {
				if( j == (int)info.idx_) {
					one_sample_prob += one_target_motion_cache_.at<double>(i, 0);
				}
				else {
					one_sample_prob += motion_cache_.at<double>(i, j);
				}
			}
			temp1[i] = one_sample_prob;
			max_log_prob = max(max_log_prob, one_sample_prob);
		}
		// old samples
		for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
			double one_sample_prob = 0.0f;
			for(int j = 0; j < motion_cache_.cols; j++) {
				one_sample_prob += motion_cache_.at<double>(i, j);
			}
			temp2[i] = one_sample_prob;
			max_log_prob = max(max_log_prob, one_sample_prob);
		}
	}
	max_log_prob -= 10;
	for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
		// prevent numerical problem...
		num += exp(temp1[i] - max_log_prob);
		denom += exp(temp2[i] - max_log_prob);
	}
#ifdef _DEBUG
	my_assert(num < DBL_MAX / 10);
	my_assert(denom < DBL_MAX / 10);
	my_assert(!isnan(num));
	my_assert(!isnan(denom));
	if(num == 0.0f || denom == 0.0f)
	{
		std::cout << "num " << num << " denom " << denom << std::endl;
		MCMCSamplePtr temp;
		print_all_cache(info, temp);
		my_assert(0);
	}
#endif
	// normalize not necessary
	return log(num) - log(denom);
}


void PriorDist::print_all_cache(const SampleInfo &info, MCMCSamplePtr sample)
{
	std::cout << "sampled type " << info.type_ << " target " << info.idx_ << std::endl;

	std::cout << "one_target" << std::endl;
	for(int row = 0; row < one_target_motion_cache_.rows; row++) {
		std::cout << setprecision(10) << one_target_motion_cache_.at<double>(row) << " ";
	}
	std::cout << std::endl;;

	std::cout << "all cachce" << std::endl;
	for(int col = 0; col < motion_cache_.cols; col++) {
		for(int row = 0; row < motion_cache_.rows; row++) {
			std::cout << setprecision(10) << motion_cache_.at<double>(row, col) << " ";
		}
		std::cout << std::endl;;
	}

	std::cout << "prev sample :: ";
	for(int row = 0; row < motion_cache_.rows; row++) {
		double one_sample_prob = 0.0;
		for(int col = 0; col < motion_cache_.cols; col++) {
			one_sample_prob += motion_cache_.at<double>(row, col);
		}
		std::cout << setprecision(50) << one_sample_prob << ":" << exp(one_sample_prob) << " ";
	}
	std::cout << std::endl;
	
	std::cout << "new sample :: ";
	for(int row = 0; row < motion_cache_.rows; row++) {
		double one_sample_prob = 0.0;
		for(int col = 0; col < motion_cache_.cols; col++) {
			if( col == (int)info.idx_) {
				one_sample_prob += one_target_motion_cache_.at<double>(row, 0);
			}
			else
				one_sample_prob += motion_cache_.at<double>(row, col);
		}
		std::cout << setprecision(50) << one_sample_prob << ":" << exp(one_sample_prob) << " ";
	}
	std::cout << std::endl;

	std::cout << "New sample at : ";
	info.state_->print();

	std::cout << "Old sample at : ";
	if(sample.get()) {
		PeopleStatePtr prev_state = sample->getState(info.idx_);
		prev_state->print();
	}

	std::cout << "prior at : ";
	std::vector<TargetDistPtr> targets = prev_dist_->getTargetDists();

	std::vector<PeopleStatePtr> states = targets[info.idx_]->getStates();
	for(size_t j = 0; j < states.size(); j++) 
		states[j]->print();
}

////////////////////////////////////////////////////////////
// Motion prior : including interactions!
////////////////////////////////////////////////////////////
void PriorDistInteract::setParameters(const std::string &name, const std::string &value)
{
	if (name == "RepulsionConstant") repulsion_const_ = boost::lexical_cast<double>(value);
	else if (name == "GroupConstant") group_const_ = boost::lexical_cast<double>(value);
	else if (name == "GroupSlopeSigmoid") group_slope_sigmoid_ = boost::lexical_cast<double>(value);
	else if (name == "GroupThresholdSigmoid") group_threshold_sigmoid_ = boost::lexical_cast<double>(value);
	else if (name == "InteractionTransitionProb") interaction_transition_prob_ = boost::lexical_cast<double>(value);
	else if (name == "InteractionOn") interaction_on_ = boost::lexical_cast<bool>(value);
	else PriorDist::setParameters(name, value);
}

cv::Mat PriorDistInteract::computeAllTargetMotionPrior(const SampleInfo &info, MCMCSamplePtr sample)
{
	cv::Mat ret;
	int num_samples = prev_dist_->getNumSamples();
	ret = cv::Mat(num_samples, 2, CV_64F);

	// need to check the info.type_ !!!!!
	if(prev_dist_type_ == "independent") {
		switch(info.type_) {
			case MoveAdd:
			case MoveDelete:
			case MoveUpdate: {
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					ret.at<double>(i, 0) = one_target_motion_cache_.at<double>(i, 0); // new prior
					ret.at<double>(i, 1) = motion_cache_.at<double>(i, (int)info.idx_); // old prior
				}
			}
			break;
			case MoveStay: {
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					MCMCSamplePtr prev_sample = prev_dist_->getSample(i);
					if(prev_sample->getExistance(info.idx_)) {
						ret.at<double>(i, 0) = log(prob_stay_);
						ret.at<double>(i, 1) = log(1 - prob_stay_);
					}
					else {
						ret.at<double>(i, 0) = log(prob_enter_);
						ret.at<double>(i, 1) = log(1 - prob_enter_);
					}
				}
			}
			break;
			case MoveLeave: {
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					MCMCSamplePtr prev_sample = prev_dist_->getSample(i);
					if(prev_sample->getExistance(info.idx_)) {
						ret.at<double>(i, 0) = log(1 - prob_stay_);
						ret.at<double>(i, 1) = log(prob_stay_);
					}
					else {
						ret.at<double>(i, 0) = log(1 - prob_enter_);
						ret.at<double>(i, 1) = log(prob_enter_);
					}
				}
			}
			break;
			default: {
				ret = cv::Scalar(0.0f); // not necessary
			}
			break;
		}

	}
	else {
		// dependent prior
		switch(info.type_) {
			case MoveAdd:
			case MoveDelete:
			case MoveStay:
			case MoveLeave:
			case MoveUpdate: {
				// new samples
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					double one_sample_prob = 0.0f;
					for(int j = 0; j < motion_cache_.cols; j++) {
						if( j == (int)info.idx_) {
							one_sample_prob += one_target_motion_cache_.at<double>(i, 0);
						}
						else {
							one_sample_prob += motion_cache_.at<double>(i, j);
						}
					}
					ret.at<double>(i, 0) = one_sample_prob;
				}
				// old samples
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					double one_sample_prob = 0.0f;
					for(int j = 0; j < motion_cache_.cols; j++) {
						one_sample_prob += motion_cache_.at<double>(i, j);
					}
					ret.at<double>(i, 1) = one_sample_prob;
				}
			}
			break;
			default: {
				// old samples
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					double one_sample_prob = 0.0f;
					for(int j = 0; j < motion_cache_.cols; j++) {
						one_sample_prob += motion_cache_.at<double>(i, j);
					}
					ret.at<double>(i, 0) = one_sample_prob;
					ret.at<double>(i, 1) = one_sample_prob;
				}
			}
			break;
		}
	}

	return ret;
}

cv::Mat PriorDistInteract::computeAllInteractionModePrior(const SampleInfo &info, MCMCSamplePtr sample)
{
	cv::Mat ret;
	int num_samples = prev_dist_->getNumSamples();
	ret = cv::Mat(num_samples, 2, CV_64F);

	if(prev_dist_type_ == "independent") { // only care about modified one
		// int idx = info.idx2_ * (info.idx2_ - 1) / 2 + info.idx_;
		switch(info.type_) {
			case MoveInteractionFlip: {
				int idx = getInteractionVariableIndex(info.idx_, info.idx2_);
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					ret.at<double>(i, 0) = one_interaction_mode_prior_cache_.at<double>(i, 0); // new prior
					ret.at<double>(i, 1) = interaction_mode_cache_.at<double>(i, idx); // old prior
				}
			}
			break;
			default: {
				ret = cv::Scalar(0.0f); // not necessary
			}
			break;
		}
	}
	else {
		// dependent prior
		switch(info.type_) {
			case MoveInteractionFlip: {
				// new samples
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					double one_sample_prob = 0.0f;

					for(int j = 0; j < sample->getNumTargets(); j++) {
						for(int k = j + 1; k < sample->getNumTargets(); k++) {
							int idx = getInteractionVariableIndex(j, k);

							if( j == (int)info.idx_ && k == (int)info.idx2_) {
								one_sample_prob += one_interaction_mode_prior_cache_.at<double>(i, 0);
							}
							else {
								one_sample_prob += interaction_mode_cache_.at<double>(i, idx);
							}
						}
					}

					ret.at<double>(i, 0) = one_sample_prob;
				}

				// old samples
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					double one_sample_prob = 0.0f;
					for(int j = 0; j < sample->getNumTargets(); j++) {
						for(int k = j + 1; k < sample->getNumTargets(); k++) {
							int idx = getInteractionVariableIndex(j, k);
							one_sample_prob += interaction_mode_cache_.at<double>(i, idx);
						}
					}
					ret.at<double>(i, 1) = one_sample_prob;
				}
			}
			break;
			default: {
				// old samples
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					double one_sample_prob = 0.0f;
					for(int j = 0; j < sample->getNumTargets(); j++) {
						for(int k = j + 1; k < sample->getNumTargets(); k++) {
							int idx = getInteractionVariableIndex(j, k);
							one_sample_prob += interaction_mode_cache_.at<double>(i, idx);
						}
					}
					ret.at<double>(i, 0) = one_sample_prob;
					ret.at<double>(i, 1) = one_sample_prob;
				}
			}
			break;
		}
	}

	return ret;
}

double PriorDistInteract::computeLogMotionPrior(const SampleInfo &info, MCMCSamplePtr sample)
{
	double ret = 0.0; 
	double num = 0.0f, denom = 0.0f;

	cv::Mat motion_prior = computeAllTargetMotionPrior(info, sample);
	cv::Mat int_mode_prior = computeAllInteractionModePrior(info, sample);
	double max_log_prob = -DBL_MAX;

	for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
		max_log_prob = max(max_log_prob, (motion_prior.at<double>(i, 0) + int_mode_prior.at<double>(i, 0)));
	};
	
	max_log_prob -= 10;
	for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
		num += exp(motion_prior.at<double>(i, 0) + int_mode_prior.at<double>(i, 0) - max_log_prob);
		denom += exp(motion_prior.at<double>(i, 1) + int_mode_prior.at<double>(i, 1) - max_log_prob);
	}

	ret = log(num) - log(denom);
#if 1
	if(interaction_on_) {
#else
	if(interaction_on_ && 
		(info.type_ == MoveUpdate || info.type_ == MoveInteractionFlip)) :
#endif
		// interactions
		ret += computeInteractionPotential(info, sample);
	}
#if 0
	if(fabs(ret) < 1e-5) {
		if(prev_dist_->getNumTargets() > info.idx_) {
			for(int i = 0; i < motion_prior.cols; i++) {
				for(int j = 0; j < motion_prior.rows; j++) {
					std::cout << motion_prior.at<double>(j, i) << " ";
				}
				std::cout << std::endl;
			}

			std::cout << "new state : " << info.state_->x_ << " " << info.state_->y_  << " " << info.state_->z_ << " "  << info.state_->vx_ << " "  << info.state_->vy_ << " "  << info.state_->vz_ << " " << std::endl;
			PeopleStatePtr state = sample->getState(info.idx_);
			std::cout << "old state : " << state->x_ << " " << state->y_  << " " << state->z_ << " "  << state->vx_ << " "  << state->vy_ << " "  << state->vz_ << " " << std::endl;
			state = prev_dist_->getTarget(info.idx_)->getMean();
			std::cout << "mean state : " << state->x_ << " " << state->y_  << " " << state->z_ << " "  << state->vx_ << " "  << state->vy_ << " "  << state->vz_ << " " << std::endl;

			std::cout << num << std::endl;
			std::cout << denom << std::endl;

			std::cout << ret << std::endl;

			assert(fabs(ret) > 1e-5);
		}
	}
#endif
	return ret;
}

void PriorDistInteract::computeNewSampletMotionPrior(const SampleInfo &info)
{
	switch(info.type_) 
	{
		case MoveAdd:
		case MoveStay:
		case MoveDelete:
		case MoveLeave:
		case MoveUpdate:
			PriorDistNS::computeNewSampletMotionPrior(info);
			break;
		case MoveInteractionFlip:
			computeNewInteractionModePrior(info);
			break;
		default:
			my_assert(0);
			break;
	}
}

void PriorDistInteract::initCache(MCMCSamplePtr sample)
{
	if(sample->getNumTargets() > 0) {
		int num_cols = getInteractionVariableIndex(0, sample->getNumTargets()); 
		// number of interaction variables
		interaction_mode_cache_ = cv::Mat(prev_dist_->getNumSamples(), num_cols, CV_64F);

		// initializat interaction mode cache
		for(int i = 0; i < sample->getNumTargets(); i++)  {
			for(int j = i + 1; j < sample->getNumTargets(); j++) {
				SampleInfo info;
				info.type_ = MoveInteractionFlip;
				info.idx_ = i;  info.idx2_ = j;
				info.group_mode_ = sample->getInteractionMode(i, j);

				computeNewInteractionModePrior(info);
				int idx = getInteractionVariableIndex(info.idx_, info.idx2_);
				for(int k = 0; k < one_interaction_mode_prior_cache_.rows; k++)  {
					interaction_mode_cache_.at<double>(k, idx) = one_interaction_mode_prior_cache_.at<double>(k, 0);
				}
			}
		}
	}

	// initialize target motion cache
	PriorDist::initCache(sample);
}

void PriorDistInteract::updateCache(const SampleInfo &info, bool accepted)
{
	if(accepted) {
		switch(info.type_) {
			case MoveAdd:
			case MoveStay:
			case MoveDelete:
			case MoveLeave:
			case MoveUpdate: {
				for(int j = 0; j < one_target_motion_cache_.rows; j++)  {
					motion_cache_.at<double>(j, info.idx_) = one_target_motion_cache_.at<double>(j, 0);
				}
			}
			break;
			case MoveInteractionFlip: {
				// int idx = info.idx2_ * (info.idx2_ - 1) / 2 + info.idx_;
				int idx = getInteractionVariableIndex(info.idx_, info.idx2_);

				my_assert(idx < interaction_mode_cache_.cols);
				for(int j = 0; j < one_interaction_mode_prior_cache_.rows; j++)  {
					interaction_mode_cache_.at<double>(j, idx) = one_interaction_mode_prior_cache_.at<double>(j, 0);
				}
			}
			break;
			default:
				my_assert(0);
			break;
		}
	}
}

// must be called with the new sample prior!!! 
void PriorDistInteract::computeNewInteractionModePrior(const SampleInfo &info)
{
	int num_samples = prev_dist_->getNumSamples();
	int num_prev_targets = prev_dist_->getNumTargets();

	one_interaction_mode_prior_cache_ = cv::Mat(num_samples, 1, CV_64F);
	one_interaction_mode_prior_cache_ = cv::Scalar(0.0);

	if(info.type_ == MoveInteractionFlip) {
		if((info.idx_ < (unsigned int)num_prev_targets ) && (info.idx2_ < (unsigned int)num_prev_targets )) {
			// both existed in the previous frame
			for(int i = 0; i < num_samples; i++) {
				MCMCSamplePtr prev_sample = prev_dist_->getSample(i);
				if( prev_sample->getExistance(info.idx_) && 
					prev_sample->getExistance(info.idx2_) ) {
					// both existed
					bool oldMode = prev_sample->getInteractionMode(info.idx_, info.idx2_);
					bool newMode = info.group_mode_;

					if(oldMode == newMode)
						one_interaction_mode_prior_cache_.at<double>(i, 0) = log(interaction_transition_prob_); 
					else
						one_interaction_mode_prior_cache_.at<double>(i, 0) = log(1 - interaction_transition_prob_); 
				}
				else {
					one_interaction_mode_prior_cache_.at<double>(i, 0) = log(0.5); 
				}
			}
		}
		else {
			// new type of interaction
			for(int i = 0; i < num_samples; i++) {
				one_interaction_mode_prior_cache_.at<double>(i, 0) = 
					log(0.5); 
			}
		}
	}
}

double PriorDistInteract::computePairwiseInteractionPotential(PeopleStatePtr state1, PeopleStatePtr state2, bool isgroup)
{
#ifdef VEL_STATE
	my_assert(state1.get() && state2.get());
	double dist = state_ground_dist(state1, state2);
	
	if(isgroup) { 
#if 1
		// group_slope_sigmoid_ = 3.0f;
		// group_threshold_sigmoid_ = 1.0f;
		// double reward = log(8.0);
		// group_const_ = 12.;
		double diff_vel = state_ground_vel_diff(state1, state2);
		double dist_sig = -log(1 + exp(group_slope_sigmoid_ * (dist - group_threshold_sigmoid_)));
		double attraction = - group_const_ * (diff_vel - 0.5);
		
		// std::cout << "interaction group potential : " << std::endl;
		// state1->print();
		// state2->print();
		// std::cout << dist_sig << " : " << attraction << std::endl;
		return dist_sig + attraction; // should there be a reward??
#else
		return 1 / ( 1 + exp(group_slope_sigmoid_ * (dist - group_threshold_sigmoid_))) 
					* exp(- group_const_ * diff_vel ); // should be added
#endif
	}
	else {
#if 0
		if(dist < .6) {
			std::cout << "interaction repulsion potential : " << std::endl;
			state1->print();
			state2->print();
			std::cout << -1 / ( repulsion_const_ * dist ) << std::endl;
		}
#endif
		return -1 / ( repulsion_const_ * dist ); // new sample
	}
#else
		// clean up the code..
		assert(0);
		return -1000000000000;
#endif
}

double PriorDistInteract::computeInteractionPotential(const SampleInfo &info, MCMCSamplePtr sample)
{
	int min_idx, max_idx;
	assert(interaction_on_);
	double ret = 0;
	PeopleStatePtr state1, state2;
	my_assert(repulsion_const_ > 0.1);
	switch(info.type_) {
		case MoveAdd:
		case MoveStay:
		case MoveUpdate:
		case MoveLeave:
		case MoveDelete: {
			// further speed up is possible(??)
			for(size_t i = 0; i < (size_t)sample->getNumTargets(); i++) {
				if(i == info.idx_) continue;
				state2 = sample->getState(i);
				// if only both targets exist. otherwise there is no interaction
				if(state2.get() == NULL) continue;

				min_idx = min((int)i, (int)info.idx_);
				max_idx = max((int)i, (int)info.idx_);

				state1 = info.state_;
				if(state1.get() != NULL) { // new sample
					ret += computePairwiseInteractionPotential(state1, state2, sample->getInteractionMode(min_idx, max_idx));
				}
				state1 = sample->getState(info.idx_);
				if(state1.get() != NULL) { // old sample
					ret -= computePairwiseInteractionPotential(state1, state2, sample->getInteractionMode(min_idx, max_idx));
				}
			}
		}
		break;

		case MoveInteractionFlip: { // real quick to compute!
			int min_idx = info.idx_;
			int max_idx = info.idx2_;

			state1 = sample->getState(info.idx_);
			state2 = sample->getState(info.idx2_);

			my_assert(info.idx_ < info.idx2_);
			my_assert(state1.get() && state2.get());

			ret += computePairwiseInteractionPotential(state1, state2, info.group_mode_);
			ret -= computePairwiseInteractionPotential(state1, state2, sample->getInteractionMode(min_idx, max_idx));
		}
		break;
		default:
			ret = 0.0;
		break;
	}

	// my_assert(ret == 0.0f);

	return ret;
}

////////////////////////////////////////////////////////////
// Motion prior : including camera and features !
////////////////////////////////////////////////////////////
void PriorDistCameraEstimate::initCache(MCMCSamplePtr sample)
{
	camera_prior_cache_ = cv::Mat(prev_dist_->getNumSamples(), 1, CV_64F);
	SampleInfo info;
	info.type_ = MoveCamUpdate;
	info.cam_state_ = sample->getCamState();
	computeNewCameraPrior(info);
	updateCache(info, true);

	feature_prior_cache_ = cv::Mat(prev_dist_->getNumSamples(), prev_dist_->getNumFeats(), CV_64F);
	// no prior for newly entered features
	for(int i = 0; i < prev_dist_->getNumFeats(); i++) {
		info.idx_ = i;
		if(sample->getFeatValidity(i)) {
			info.type_ = MoveFeatStay;
			info.feat_state_ = sample->getFeatState(i);
		}
		else {
			info.type_ = MoveFeatLeave;
		}
		computeNewFeaturePrior(info);
		updateCache(info, true);
	}

	PriorDistInteract::initCache(sample);
}

void PriorDistCameraEstimate::updateCache(const SampleInfo &info, bool accepted)
{
	if(accepted) {
		switch(info.type_) {
			case MoveCamUpdate:
				for(int i = 0; i < camera_prior_cache_.rows; i++)  {
					camera_prior_cache_.at<double>(i, 0) = new_camera_prior_cache_.at<double>(i, 0);
				}
				break;
			case MoveFeatStay:
			case MoveFeatLeave:
			case MoveFeatUpdate:
				if(info.idx_ < (unsigned int)prev_dist_->getNumFeats()) {
					for(int i = 0; i < feature_prior_cache_.rows; i++)  {
						feature_prior_cache_.at<double>(i, info.idx_) = one_feature_prior_cache_.at<double>(i, 0);
					}
				}
				break;
			default:
				PriorDistInteract::updateCache(info, accepted);
				break;
		}
	}
}

void PriorDistCameraEstimate::setParameters(const std::string &name, const std::string &value)
{
	if (name == "CameraMotionSigmaX")  camera_motion_sigma_x_ = boost::lexical_cast<double>(value);
	else if (name == "CameraMotionSigmaZ")  camera_motion_sigma_z_ = boost::lexical_cast<double>(value);
	else if (name == "CameraMotionSigmaYaw")  camera_motion_sigma_yaw_ = boost::lexical_cast<double>(value);
	else if (name == "CameraMotionSigmaV")  camera_motion_sigma_v_ = boost::lexical_cast<double>(value);
	else if (name == "CameraMotionSigmaHorizon")  camera_motion_sigma_horizon_ = boost::lexical_cast<double>(value);
	else if (name == "ProbFeatEnter")  prob_feat_enter_ = boost::lexical_cast<double>(value);
	else if (name == "ProbFeatStay")  prob_feat_stay_ = boost::lexical_cast<double>(value);
	else PriorDistInteract::setParameters(name, value);
}

void PriorDistCameraEstimate::computeNewSampletMotionPrior(const SampleInfo &info)
{
	switch(info.type_) 
	{
		case MoveCamUpdate:
			computeNewCameraPrior(info);
			break;
		case MoveFeatStay:
		case MoveFeatLeave:
		case MoveFeatUpdate:
			computeNewFeaturePrior(info);
			break;
		default:
			PriorDistInteract::computeNewSampletMotionPrior(info);
			break;
	}
}


double PriorDistCameraEstimate::computeLogMotionPrior(const SampleInfo &info, MCMCSamplePtr sample)
{
	double ret = 0.0; 
	double num = 0.0f, denom = 0.0f;

	cv::Mat motion_prior = computeAllTargetMotionPrior(info, sample);
	cv::Mat int_mode_prior = computeAllInteractionModePrior(info, sample);
	cv::Mat feat_prior = computeAllFeatureMotionPrior(info, sample);
	cv::Mat cam_prior = computeCameraMotionPrior(info, sample);

	double max_log_prob = -DBL_MAX;
	// find max for numerical stability
	// and use cam_prior as a summation buffer
	for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
		cam_prior.at<double>(i, 0) += motion_prior.at<double>(i, 0)
										+ int_mode_prior.at<double>(i, 0)
										+ feat_prior.at<double>(i, 0);

		cam_prior.at<double>(i, 1) += motion_prior.at<double>(i, 1)
										+ int_mode_prior.at<double>(i, 1)
										+ feat_prior.at<double>(i, 1);

		max_log_prob = max(max_log_prob, cam_prior.at<double>(i, 0));
	};
	
	max_log_prob -= 10;
	for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
		num += exp(cam_prior.at<double>(i, 0) - max_log_prob);
		denom += exp(cam_prior.at<double>(i, 1) - max_log_prob);
	}
	ret = log(num) - log(denom);
	///////////////////////////////////////////////////////
	// TODO : uncomment below to use interaction potential
	///////////////////////////////////////////////////////
	// compute the difference of all affected pairwist potentials
#if 1
	if(interaction_on_) {
#else
	if(interaction_on_ && 
		(info.type_ == MoveUpdate || info.type_ == MoveInteractionFlip)) {
#endif
		ret += computeInteractionPotential(info, sample);
	}
#if 0 // def HEIGHT_PRIOR
	// compute the height prior
	double mheight = 1.7;
	double sheight = 0.1;
	double thheight = 2;

	switch(info.type_) {
		case MoveAdd:
		case MoveStay: {
			double height = info.state_->getY();
			ret +=  pow(thheight, 2) - pow((height - mheight) / sheight, 2);
		}
		break;
		case MoveLeave:
		case MoveDelete: {
			double height = sample->getState(info.idx_)->getY();
			ret -=  pow(thheight, 2) - pow((height - mheight) / sheight, 2);
		}
		break;
		case MoveUpdate: {
			double newh = info.state_->getY();
			double oldh = sample->getState(info.idx_)->getY();

			ret += - pow((newh - mheight) / sheight, 2)
				 + pow((oldh - mheight) / sheight, 2);
		}
		break;
		default:
		break;
	}
#endif
#if 0
	if(fabs(ret) < 1e-5) {
		if(prev_dist_->getNumTargets() > info.idx_) {
			for(int i = 0; i < motion_prior.cols; i++) {
				for(int j = 0; j < motion_prior.rows; j++) {
					std::cout << motion_prior.at<double>(j, i) << " ";
				}
				std::cout << std::endl;
			}

			std::cout << "new state : " << info.state_->x_ << " " << info.state_->y_  << " " << info.state_->z_ << " "  << info.state_->vx_ << " "  << info.state_->vy_ << " "  << info.state_->vz_ << " " << std::endl;
			PeopleStatePtr state = sample->getState(info.idx_);
			std::cout << "old state : " << state->x_ << " " << state->y_  << " " << state->z_ << " "  << state->vx_ << " "  << state->vy_ << " "  << state->vz_ << " " << std::endl;
			state = prev_dist_->getTarget(info.idx_)->getMean();
			std::cout << "mean state : " << state->x_ << " " << state->y_  << " " << state->z_ << " "  << state->vx_ << " "  << state->vy_ << " "  << state->vz_ << " " << std::endl;

			std::cout << num << std::endl;
			std::cout << denom << std::endl;

			std::cout << ret << std::endl;

			assert(fabs(ret) > 1e-5);
		}
	}
#endif
	return ret;
}

cv::Mat PriorDistCameraEstimate::computeAllFeatureMotionPrior(const SampleInfo &info, MCMCSamplePtr sample)
{
	cv::Mat ret; 
	int num_samples = prev_dist_->getNumSamples();
	ret = cv::Mat(num_samples, 2, CV_64F);

	// need to check the info.type_ !!!!!
	if(prev_dist_type_ == "independent") {
		// independent prior
		switch(info.type_) {
			case MoveFeatStay:
			case MoveFeatLeave:
			case MoveFeatUpdate: {
				if(info.idx_ < (unsigned int)prev_dist_->getNumFeats()) {
					for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
						ret.at<double>(i, 0) = one_feature_prior_cache_.at<double>(i, 0); // new prior
						ret.at<double>(i, 1) = feature_prior_cache_.at<double>(i, (int)info.idx_); // old prior
					}
				}
				else {
					// no prior for newly entered features
					ret = cv::Scalar(0.0f);
				}
			}
			break;
			default: {
				ret = cv::Scalar(0.0f); // not necessary
			}
			break;
		}
	}
	else {
		// dependent prior
		switch(info.type_) {
			case MoveFeatStay:
			case MoveFeatLeave:
			case MoveFeatUpdate: {
				// new samples
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					double one_sample_prob = 0.0f;
					for(int j = 0; j < feature_prior_cache_.cols; j++) {
						if( j == (int)info.idx_) {
							one_sample_prob += one_feature_prior_cache_.at<double>(i, 0);
						}
						else {
							one_sample_prob += feature_prior_cache_.at<double>(i, j);
						}
					}
					ret.at<double>(i, 0) = one_sample_prob;
				}
				// old samples
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					double one_sample_prob = 0.0f;
					for(int j = 0; j < feature_prior_cache_.cols; j++) {
						one_sample_prob += feature_prior_cache_.at<double>(i, j);
					}
					ret.at<double>(i, 1) = one_sample_prob;
				}
			}
			break;
			default: {
				// old samples
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					double one_sample_prob = 0.0f;
					for(int j = 0; j < feature_prior_cache_.cols; j++) {
						one_sample_prob += feature_prior_cache_.at<double>(i, j);
					}
					ret.at<double>(i, 0) = one_sample_prob;
					ret.at<double>(i, 1) = one_sample_prob;
				}
			}
			break;
		}
	}

	return ret;
}

cv::Mat PriorDistCameraEstimate::computeCameraMotionPrior(const SampleInfo &info, MCMCSamplePtr sample)
{
	cv::Mat ret; 
	int num_samples = prev_dist_->getNumSamples();
	ret = cv::Mat(num_samples, 2, CV_64F);

	// need to check the info.type_ !!!!!
	if(prev_dist_type_ == "independent") {
		// independent prior
		switch(info.type_) {
			case MoveCamUpdate: {
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					ret.at<double>(i, 0) = new_camera_prior_cache_.at<double>(i, 0); // new prior
					ret.at<double>(i, 1) = camera_prior_cache_.at<double>(i, 0); // old prior
				}
			}
			break;
			default: {
				ret = cv::Scalar(0.0f); // not necessary
			}
			break;
		}
	}
	else {
		// dependent prior
		switch(info.type_) {
			case MoveCamUpdate: {
				// new samples
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					ret.at<double>(i, 0) = new_camera_prior_cache_.at<double>(i, 0); // new prior
					ret.at<double>(i, 1) = camera_prior_cache_.at<double>(i, 0); // old prior
				}
			}
			break;
			default: {
				// old samples
				for(int i = 0; i < prev_dist_->getNumSamples(); i++) {
					ret.at<double>(i, 0) = ret.at<double>(i, 1) = camera_prior_cache_.at<double>(i, 0);
				}
			}
			break;
		}
	}

	return ret;
}

#define PROB_WEIGHT_FEAT_DIST	-100.0f
#define PROB_FEAT_ENTER_CONST	-2.0f
void PriorDistCameraEstimate::computeNewFeaturePrior(const SampleInfo &info)
{
	int num_samples = prev_dist_->getNumSamples();
	one_feature_prior_cache_ = cv::Mat(num_samples, 1, CV_64F);
	one_feature_prior_cache_ = cv::Scalar(0.0);

	my_assert((info.type_ == MoveFeatStay)
				|| (info.type_ == MoveFeatLeave)
				|| (info.type_ == MoveFeatUpdate));

	if(info.idx_ < (unsigned int)feature_prior_cache_.cols) {
		if((info.type_ == MoveFeatStay) || (info.type_ == MoveFeatUpdate)) {
			for(int i = 0; i < num_samples; i++) {
				MCMCSamplePtr prev_sample = prev_dist_->getSample(i);
				if(prev_sample->getFeatValidity(info.idx_)) {
					one_feature_prior_cache_.at<double>(i, 0) = 
						log(prob_feat_stay_);
						// + PROB_WEIGHT_FEAT_DIST * feat_state_dist(prev_sample->getFeatState(info.idx_), info.feat_state_);
				}
				else {
					one_feature_prior_cache_.at<double>(i, 0) = 
						log(prob_feat_enter_); // + PROB_FEAT_ENTER_CONST;
				}
			}
		}
		else {
			my_assert(info.type_ == MoveFeatLeave);
			for(int i = 0; i < num_samples; i++) {
				MCMCSamplePtr prev_sample = prev_dist_->getSample(i);
				if(prev_sample->getFeatValidity(info.idx_)) {
					one_feature_prior_cache_.at<double>(i, 0) = 
						log(1 - prob_feat_stay_);
				}
				else {
					one_feature_prior_cache_.at<double>(i, 0) = 
						log(1 - prob_feat_enter_); // + PROB_FEAT_ENTER_CONST;
				}
			}
		}
	}
	else {
		// newly entered feature
		// do nothing. there is no prior. everything's are equally possible
		// only dependent on observation
	}
}

void PriorDistCameraEstimate::computeNewCameraPrior(const SampleInfo &info)
{
	int num_samples = prev_dist_->getNumSamples();
	new_camera_prior_cache_ = cv::Mat(num_samples, 1, CV_64F);
	new_camera_prior_cache_ = cv::Scalar(0.0);

	my_assert(info.type_ == MoveCamUpdate);
	double dt = timestamp_ - prev_dist_->getTimeStamp();

	CamStatePtr state = info.cam_state_;
	for(int i = 0; i < num_samples; i++) {
		MCMCSamplePtr prev_sample = prev_dist_->getSample(i);
		CamStatePtr prev_state = prev_sample->getCamState();
		if(prev_state.get() == NULL) {
			my_assert(num_samples <= 2);
			// no camera prior at all. It's the first frame
			new_camera_prior_cache_.at<double>(i, 0) = 0.0;
			continue;
		}
#if 1
		double dx, dz;
		dx = -state->getV() * sin(state->getYaw()) * dt;
		dz = state->getV() * cos(state->getYaw()) * dt;
		new_camera_prior_cache_.at<double>(i, 0) = 
			log_gaussian_prob(state->getX() - dx, prev_state->getX(), camera_motion_sigma_x_ * dt)
			+ log_gaussian_prob(state->getZ() - dz, prev_state->getZ(), camera_motion_sigma_z_ * dt)
			+ log_gaussian_prob(state->getYaw(), prev_state->getYaw(), camera_motion_sigma_yaw_ * dt)
			+ log_gaussian_prob(state->getHorizon(), prev_state->getHorizon(), camera_motion_sigma_horizon_ * dt)
			+ log_gaussian_prob(state->getV(), prev_state->getV(), camera_motion_sigma_v_ * dt);
		// avoid negative velocity
		if(state->getV() < 0) new_camera_prior_cache_.at<double>(i, 0) -= 100;
#if 0	
		std::cout	
			<< "x std " << camera_motion_sigma_x_ * dt
			<< "z std " << camera_motion_sigma_z_ * dt
			<< "yaw std " << camera_motion_sigma_yaw_ * dt
			<< "horizon std " << camera_motion_sigma_horizon_ * dt
			<< "v std " << camera_motion_sigma_v_ * dt << std::endl;

		std::cout << "new_cam ";
		state->print();
		std::cout << "prev_cam ";
		prev_state->print();
		std::cout << "log_prob : " << new_camera_prior_cache_.at<double>(i, 0) << std::endl;
		assert(0);
#endif
#else
		new_camera_prior_cache_.at<double>(i, 0) = 
			log_gaussian_prob(state->getX(), prev_state->getX(), camera_motion_sigma_x_ * dt)
			+ log_gaussian_prob(state->getZ(), prev_state->getZ(), camera_motion_sigma_z_ * dt)
			+ log_gaussian_prob(state->getYaw(), prev_state->getYaw(), camera_motion_sigma_yaw_ * dt);
#endif
	}
}
