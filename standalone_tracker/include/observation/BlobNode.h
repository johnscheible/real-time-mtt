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

#ifndef _BLOB_NODE_H_
#define _BLOB_NODE_H_

#include <observation/ObservationNode.h>
#include <unordered_map>

 namespace people {
  class BlobNode : public ObservationNode 
  {
  public:
    BlobNode();
    virtual ~BlobNode();

    virtual void setParameter(const std::string &name, const std::string &value);
    virtual void setData(const void *data, const std::string &type);
    virtual double getConfidence(const cv::Rect &rt, double depth = 0);

    virtual void preprocess();
    virtual std::vector<cv::Rect> getDetections();

    void quaryData(const std::string &name, void *data);
  private:
    bool supress(std::vector<cv::KeyPoint>& keypoints, cv::KeyPoint& query);
    bool detectBlobs();
    cv::Ptr<cv::FeatureDetector> default_detector_;
	int horizon_row_;

  protected:
    cv::Mat *color_image_;
    double time_sec_;
    
    // detections
    std::vector<cv::Rect> found_;
    std::vector<double> responses_;
	std::unordered_map<const cv::Rect*,double> confidence_;

    // parameters
    double hit_threshold_;
    double det_scale_;

    double det_std_x_;
    double det_std_y_;
    double det_std_h_;
    
    cv::Size imsize_;
    double pos_threshold_;

    // Detector threshold
	vector<int> thresh_vals;

    int version_;
  };
}; // Namespace

#endif
