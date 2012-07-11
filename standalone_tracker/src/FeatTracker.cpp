#include <sys/time.h>
#include <common/util.h>
#include <common/FeatTracker.h>

namespace people {

void draw_key_points(cv::Mat &img, std::vector<cv::KeyPoint> &points)
{
	for(size_t i = 0; i < points.size(); i++) {
		cv::KeyPoint pt = points[i];
		cv::circle(img, pt.pt, 1 /*pt.size*/, cv::Scalar(0, 255, 0), 2);
	}
}

FeatTracker::FeatTracker()
{
	detector_type_ = "SURF";

	max_features_ = 300;
	max_flow_dist_ = 0.1;
	show_image_ = false;
}

FeatTracker::~FeatTracker()
{
}

void FeatTracker::setVideoFile(const std::string &video_file)
{
	if(!video_.isOpened()) {
		int frame_rate = 2;
		if(!video_.open(video_file, CV_FOURCC('X','V','I','D'), frame_rate, cv::Size(640 , 480), true))
		assert(video_.isOpened());
	}
}

void FeatTracker::setDetectorType(const std::string &type)
{
	/* type of detectors supported by opencv
	"FAST" – FastFeatureDetector
	"STAR" – StarFeatureDetector
	"SIFT" – SiftFeatureDetector
	"SURF" – SurfFeatureDetector
	"ORB" – OrbFeatureDetector
	"MSER" – MserFeatureDetector
	"GFTT" – GoodFeaturesToTrackDetector
	"HARRIS" – GoodFeaturesToTrackDetector with Harris detector enabled
	"Dense" – DenseFeatureDetector
	"SimpleBlob" – SimpleBlobDetector

	Also a combined format is supported: feature detector adapter name ( "Grid" – GridAdaptedFeatureDetector, "Pyramid" – PyramidAdaptedFeatureDetector ) 
										+ feature detector name (see above), for example: "GridFAST", "PyramidSTAR" .
	*/
	detector_type_ = type;
}

void FeatTracker::setNewImage(cv::Mat &image, double timestamp)
{
	prev_image_ = current_image_;
	current_image_ = image;

	prev_timestamp_ = current_timestamp_;
	current_timestamp_ = timestamp;
}
/*
void FeatTracker::get_features(double timestamp, std::vector<cv::Point2f> &pts, std::vector<int> &index)
{
	index.clear();
	pts.clear();

	// naive implementation... maybe too slow if there are too many features?
	// loop over tracks
	for(size_t i = 0; i < feat_tracks_.size(); i++) {
		imfeat_track track = feat_tracks_[i];
		// loop over timestamp
		for(size_t j = 0; j < track.feats_.size(); j++) {
			if(track.feats_[j].timestamp_ == timestamp) {
				pts.push_back(track.feats_[j].pt_);
				index.push_back(i);
				break;
			}
		}
	}
}
*/
void FeatTracker::get_features(double timestamp, std::vector<cv::Point2f> &pts, std::vector<float> &responses, std::vector<int> &index)
{
	index.clear();
	pts.clear();
	responses.clear();
	// naive implementation... maybe too slow if there are too many features?
	// loop over tracks
	for(size_t i = 0; i < feat_tracks_.size(); i++) {
		imfeat_track track = feat_tracks_[i];
		// loop over timestamp
		for(int j = track.feats_.size() - 1; j >= 0; j--) {
			if(track.feats_[j].timestamp_ == timestamp) {
				pts.push_back(track.feats_[j].pt_);
				index.push_back(i);
				responses.push_back((float)track.response_);
				break;
			}
		}
	}

}

std::vector<cv::KeyPoint> FeatTracker::detect_features(cv::Mat &image)
{
	std::vector<cv::KeyPoint> key_points;
#if 1
	if(detector_type_ == "SURF") {
		cv::SurfFeatureDetector det(25);
		det.detect(image, key_points);
	}
	else {
		assert(0);
	}
#else
	cv::Ptr<cv::FeatureDetector> det = cv::FeatureDetector::create(detector_type_);
	det->detect(image, key_points);
#endif
#if 0
	std::cout << "detected " << key_points.size() << " number of features!!!" << std::endl;
	cv::Mat temp_img;
	cvtColor(image, temp_img, CV_GRAY2BGR);
	
	draw_key_points(temp_img, key_points);
	
	cv::imshow("new features", temp_img);
	cv::waitKey(2000);
#endif
	return key_points;
}

bool KeyPointResponseCompare(cv::KeyPoint a, cv::KeyPoint b) 
{ 
	// decreasing order
	return (a.response > b.response); 
}

#define MARGIN	0.15
bool FeatTracker::isValidGroundFeature(cv::Point2f pt)
{
	bool ret = (pt.x > current_image_.cols * MARGIN) && (pt.x < current_image_.cols * (1 - MARGIN));
	ret = ret && (pt.y > current_image_.rows * .0 ) && (pt.y < current_image_.rows * 0.7);
	return ret;
}

void FeatTracker::processTracking()
{
	// obtain last frame features
	std::vector<cv::Point2f> 	pts;
	std::vector<int> 			indices;
	std::vector<float> 			responses;

	get_features(prev_timestamp_, pts, responses, indices);

	int ntracked = 0;
	std::vector<cv::Point2f> npts;
	std::vector<uchar> status;
	std::vector<float> err;
	std::vector<bool> valid;

	struct timeval timeofday;
	double rt0, rt1, rt2, rt3;
	gettimeofday(&timeofday,NULL); rt0 = timeofday.tv_sec + 1e-6 * timeofday.tv_usec;
	// track existing features!!
	if(pts.size() > 0) {
		cv::calcOpticalFlowPyrLK(prev_image_, current_image_, pts, npts, status, err, cv::Size(current_image_.cols / 50, current_image_.cols / 50), 3);
		
		assert(pts.size() == npts.size());
		assert(pts.size() == status.size());
		assert(pts.size() == err.size());

		for(size_t i = 0; i < pts.size(); i++) {
			float dist = sqrt(pow(pts[i].x - npts[i].x, 2) + pow(pts[i].y - npts[i].y, 2));
			valid.push_back(status[i] == 1 && dist < (max_flow_dist_ * current_image_.cols) && isValidGroundFeature(npts[i]));
			if(valid[i]) { // succeeded to track
				// insert new location to feature track
				imfeat feat;
				feat.timestamp_ = current_timestamp_;
				feat.pt_ = npts[i];

				feat_tracks_[indices[i]].feats_.push_back(feat);
				// std::cout << "[" << pts[i].x << ", "<< pts[i].y << "=>" << npts[i].x << ", " <<npts[i].y << "]";
				ntracked++;
			}
		}
		// cv::waitKey();
	}
	gettimeofday(&timeofday,NULL); rt1 = timeofday.tv_sec + 1e-6 * timeofday.tv_usec;
	if(ntracked < max_features_) {
		// detect new features
		std::vector<cv::KeyPoint> key_points = detect_features(current_image_);
		// sort wrt response value
		std::sort(key_points.begin(), key_points.end(), KeyPointResponseCompare);
		// identify new features to be added
		int i = 0;
		std::cout << "Adding new features!!!!! : tracked : " << ntracked << std::endl;
		while(ntracked < max_features_) {
			// check distance, see if the feature is far enough from all existing tracks to avoid overlapping features..
			float min_dist = 1000.0;
			if(i >= (int)key_points.size()) break;
			// my_assert(key_points[i].pt.x < current_image_.cols);
			// my_assert(key_points[i].pt.y < current_image_.rows);
			
			if(!isValidGroundFeature(key_points[i].pt)) {
				i++; 
				continue;
			}

			for(size_t temp = 0; temp < indices.size(); temp++) {
				size_t j = indices[temp];
			// for(size_t j = 0; j < feat_tracks_.size(); j++) {
				imfeat_track track = feat_tracks_[j];
				imfeat feat = track.feats_[track.feats_.size() - 1]; // the last feature
				if (feat.timestamp_ == current_timestamp_) { // 
					float dist = sqrt(pow(key_points[i].pt.x - feat.pt_.x, 2) + pow(key_points[i].pt.y - feat.pt_.y, 2));
					if(dist < min_dist) {
						min_dist = dist;
					}
				}
			}

			if(min_dist > (current_image_.cols / 25)) {
				imfeat_track new_track;
				imfeat feat;
				feat.pt_ = key_points[i].pt;
				feat.timestamp_ = current_timestamp_;
				new_track.feats_.push_back(feat);
				new_track.response_ = key_points[i].response;
				feat_tracks_.push_back(new_track);

				indices.push_back(feat_tracks_.size() - 1);

				ntracked++;
			}

			i++;
		}
	}
	std::cout << "Added new features!!!!! : tracked : " << ntracked << std::endl;
	gettimeofday(&timeofday,NULL); rt2 = timeofday.tv_sec + 1e-6 * timeofday.tv_usec;
	if (show_image_ || video_.isOpened()) {
		get_features(current_timestamp_, pts, responses, indices);

		cv::Mat temp_img;
		cvtColor(current_image_, temp_img, CV_GRAY2BGR);
		
		std::cout << "showing total " << pts.size() << " number of features" << std::endl;
		for(size_t i = 0; i < pts.size(); i++) {
			cv::Point2f pt = pts[i];

			cv::circle(temp_img, pt, 1 /*pt.size*/, cv::Scalar(255, 0, 0), 2);
			
			int idx = indices[i];
			imfeat_track feat_track = feat_tracks_[idx];
			for(size_t j = 1; j < 20; j++) {
				if((int)feat_track.feats_.size() - (int)j - 1 < 0) { break; }
				cv::line(temp_img, feat_track.feats_[feat_track.feats_.size() - j].pt_, feat_track.feats_[feat_track.feats_.size() - j - 1].pt_, cv::Scalar(150, 0, 0), 2);
			}
		}

		if(video_.isOpened()) {
			video_ << temp_img;
		}
		
		if(show_image_) {
			cv::imshow("feat_tracks", temp_img);
			cv::waitKey(20);
		}
	}

	gettimeofday(&timeofday,NULL); rt3 = timeofday.tv_sec + 1e-6 * timeofday.tv_usec;
	std::cout << "run time : tracking " << rt1 - rt0 << " detection " << rt2 - rt1 << " visualization " << rt3 - rt2 << std::endl;
}
};
