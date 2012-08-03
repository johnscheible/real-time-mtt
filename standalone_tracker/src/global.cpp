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

#include <common/global.h>
#include <common/util.h>
namespace people {
	cv::RNG	g_rng((double)time(NULL));
	ObjectType g_objtype = ObjPerson;

	// ObjectStateType g_obj_state_type_ = ObjectStateTypeObjectVel;
	// FeatureStateType g_feat_state_type_ = FeatureStateTypeStatic;
	// CameraStateType g_cam_state_type_ = CameraStateTypeSimplified;

	double gaussian_prob(double x, double m, double std)
	{
		double var = std * std;

		if(std == 0) return 1.0;

		return 1 / sqrt(2 * M_PI * var) * exp(- pow(x - m, 2) / (2 * var));
	}

	double log_gaussian_prob(double x, double m, double std)
	{
		if(std == 0) return 0.0; // no effect!
		return -log(sqrt(2 * M_PI) * std) - ( pow((x - m) / std, 2) / 2.0 );
	}
	
	double log_gaussian_prob(cv::Mat &x, cv::Mat& m, cv::Mat &icov, double det)
	{
		my_assert(x.rows == m.rows);
		my_assert(x.rows == icov.cols);
		
		cv::Mat dx = x - m;
		cv::Mat temp = ( (dx.t() * icov) * dx  / 2.0 );
		
		my_assert(temp.rows == 1 && temp.cols == 1);

		double ret =  - x.rows * log(2 * M_PI) / 2 
			   - log(det) / 2  - temp.at<double>(0, 0);

		return ret;
	}


};
