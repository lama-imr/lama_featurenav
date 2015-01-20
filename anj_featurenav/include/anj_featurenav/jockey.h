/* Implements a feature-based learning and navigating jockey based on free
 * OpenCV feature descriptor and matcher.
 * 
 * The package defines the feature extractor and the feature matcher
 * functions required by the featurenav_base package to obtain a working couple
 * learning/navigating jockeys. The words feature and descriptor are used as
 * synonymous.
 *
 * Parameters:
 * - name, type, default value, description
 * - ~feature_detector/type, String, "FAST", algorithm for feature detection.
 *     Can be one of "FAST", "STAR", "ORB", "MSER", "GFTT", "Dense",
 *     "SimpleBlob". They make use of their corresponding algorithm in the
 *     OpenCV library (for example "FAST" implies the use of
 *     "cv::FastFeatureDetector").
 * - ~descriptor_extractor/type, String, "BRIEF", algorithm for feature
 *     extraction. Can be one of "ORB", "BRIEF". They make use of their
 *     corresponding algorithm in the OpenCV library (for example, "BRIEF"
 *     implies the use of "cv::BriefDescriptorExtractor").
 * - ~descriptor_matcher/type, String, "FlannBased", algorithm for feature
 *     matching. Can be one of "BruteForce", "FlannBased". They make use of
 *     their corresponding algorithm in the OpenCV library (for example,
 *     "FlannBased" implies the use of "cv::FlannBasedMatcher").
 *
 * Parameters for "FAST":
 * cf. http://docs.opencv.org/modules/features2d/doc/feature_detection_and_description.html#fast
 * - ~feature_detector/threshold, Int, 1, cf. OpenCV documentation.
 * - ~feature_detector/nonmax_suppression, Bool, true, cf. OpenCV documentation.
 *
 * Parameters for "STAR":
 * cf. http://docs.opencv.org/modules/features2d/doc/common_interfaces_of_feature_detectors.html?highlight=star#starfeaturedetector
 * - ~feature_detector/max_size, Int, 16, cf. OpenCV documentation.
 * - ~feature_detector/response_threshold, Int, 30, cf. OpenCV documentation.
 * - ~feature_detector/line_threshold_projected, Int, 10, cf. OpenCV documentation.
 * - ~feature_detector/line_threshold_binarized, Int, 8, cf. OpenCV documentation.
 * - ~feature_detector/suppress_nonmax_size, Int, 5, cf. OpenCV documentation.
 *
 * Parameters for "ORB" (as feature detector):
 * cf. http://docs.opencv.org/modules/features2d/doc/feature_detection_and_description.html#orb
 * - ~feature_detector/scale_factor, Float, 1.2, cf. OpenCV documentation.
 * - ~feature_detector/n_features, Int, 500, cf. OpenCV documentation.
 * - ~feature_detector/n_levels, Int, 8, cf. OpenCV documentation.
 * - ~feature_detector/edge_threshold, Int, 31, cf. OpenCV documentation.
 * - ~feature_detector/first_level, Int, 0, cf. OpenCV documentation.
 * - ~feature_detector/wta_k, Int, 2, cf. OpenCV documentation.
 * - ~feature_detector/score_type, Int, 0 (for HARRIS_SCORE), 0 for Harris
 *     score, 1 for FAST score, cf. OpenCV documentation.
 * - ~feature_detector/patch_size, Int, 31, cf. OpenCV documentation.
 *
 * Parameters for "MSER":
 * cf. http://docs.opencv.org/modules/features2d/doc/feature_detection_and_description.html#mser
 * - ~feature_detector/delta, Int, 5, cf. OpenCV documentation.
 * - ~feature_detector/min_area, Int, 60, cf. OpenCV documentation.
 * - ~feature_detector/max_area, Int, 14400, cf. OpenCV documentation.
 * - ~feature_detector/max_variation, Float, 0.25, cf. OpenCV documentation.
 * - ~feature_detector/min_diversity, Float, 0.2, cf. OpenCV documentation.
 * - ~feature_detector/max_evolution, Int, 200, cf. OpenCV documentation.
 * - ~feature_detector/area_threshold, Float, 1.01, cf. OpenCV documentation.
 * - ~feature_detector/min_margin, Float, 0.003, cf. OpenCV documentation.
 * - ~feature_detector/edge_blur_size, Int, 5, cf. OpenCV documentation.
 *
 * Parameters for "GFTT":
 * cf. http://docs.opencv.org/modules/features2d/doc/common_interfaces_of_feature_detectors.html?highlight=gftt#goodfeaturestotrackdetector
 * - ~feature_detector/max_corners, 1000, Int, cf. OpenCV documentation.
 * - ~feature_detector/block_size, 3, Int, cf. OpenCV documentation.
 * - ~feature_detector/quality_level, Float, 0.01, cf. OpenCV documentation.
 * - ~feature_detector/min_distance, Float, 1, cf. OpenCV documentation.
 * - ~feature_detector/k, Float, 0.04, cf. OpenCV documentation.
 * - ~feature_detector/use_harris_detector, Bool, false, cf. OpenCV documentation.
 *
 * Parameters for "Dense":
 * cf. http://docs.opencv.org/modules/features2d/doc/common_interfaces_of_feature_detectors.html?highlight=gftt#densefeaturedetector
 * - ~feature_detector/feature_scale_levels, Int, 1, cf. OpenCV documentation.
 * - ~feature_detector/init_xy_step, Int, 6, cf. OpenCV documentation.
 * - ~feature_detector/init_img_bound, Int, 0, cf. OpenCV documentation.
 * - ~feature_detector/init_feature_scale, Float, 1, cf. OpenCV documentation.
 * - ~feature_detector/feature_scale_mul, Float, 0.1, cf. OpenCV documentation.
 * - ~feature_detector/vary_xy_step_with_scale, Bool, true, cf. OpenCV documentation.
 * - ~feature_detector/vary_img_bound_with_scale, Bool, false, cf. OpenCV documentation.
 *
 * Parameters for "ORB" (as feature extractor):
 * cf. http://docs.opencv.org/modules/features2d/doc/feature_detection_and_description.html#orb
 * same parameters as "ORB" above but replacing "feature_detector" with "descriptor_extractor"
 *
 * Parameters for "BRIEF":
 * cf. http://docs.opencv.org/modules/features2d/doc/common_interfaces_of_descriptor_extractors.html?highlight=brief#briefdescriptorextractor
 * - ~descriptor_extractor/bytes, Int, 32, cf. OpenCV documentation.
 *
 * Parameters for "FlannBased":
 * cf. http://docs.opencv.org/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html?highlight=flannbasedmatcher#flannbasedmatcher
 * none
 * 
 * Parameters for "BruteForce":
 * cf. http://docs.opencv.org/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html#bfmatcher
 * - ~descriptor_matcher/norm, String, "L2", one of "L1", "L2", "HAMMING", "HAMMING2", cf. OpenCV documentation.
 * - ~descriptor_matcher/cross_check, Bool, false, cf. OpenCV documentation.
 */

#ifndef ALJ_FEATURENAV_JOCKEY_H
#define ALJ_FEATURENAV_JOCKEY_H

#include <algorithm>
#include <cassert>
#include <vector>

#include <boost/scoped_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/features2d/features2d.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>

#include <featurenav_base/anjockey.h>
#include <featurenav_base/Feature.h>

namespace anj_featurenav
{

using std::vector;
using cv::KeyPoint;
using cv::DMatch;
using featurenav_base::Feature;

class Jockey
{
  public:

    Jockey(const std::string& name);

    std::string getLearningJockeyName() const;
    std::string getNavigatingJockeyName() const;

  private:

    void initDetectorFast();
    void initDetectorStar();
    void initDetectorOrb();
    void initDetectorMser();
    void initDetectorGftt();
    void initDetectorDense();
    void initDetectorSimpleblob();
    void initExtractorOrb();
    void initExtractorBrief();
    void initMatcherBruteforce();
    void initMatcherFlannbased();

    void extractFeatures(const sensor_msgs::ImageConstPtr& image, vector<KeyPoint>& keypoints, vector<Feature>& descriptors) const;
    void matchDescriptors(const vector<Feature>& query_descriptors, const vector<Feature>& train_descriptors, vector<vector<DMatch> >& matches) const;

    // ROS parameters, shown outside.
    std::string feature_detector_type_;
    std::string descriptor_extractor_type_;
    std::string descriptor_matcher_type_;

    // Internals.
    ros::NodeHandle private_nh_;
    std::string base_name_;  //!> The base name for learning and navigating jockeys.
    boost::scoped_ptr<featurenav_base::ANJockey> anjockey_ptr_;
    boost::scoped_ptr<cv::FeatureDetector> feature_detector_;
    boost::scoped_ptr<cv::DescriptorExtractor> descriptor_extractor_;
    boost::scoped_ptr<cv::DescriptorMatcher> descriptor_matcher_;
    std::string feature_detector_code_;
    std::string descriptor_extractor_code_;
    std::string descriptor_matcher_code_;
    std::string map_interface_name_;  //!> Map interface name for Segment messages.
};
  
inline Feature rosFromCv(const cv::Mat& descriptor)
{
  Feature ros_descriptor;
  if (descriptor.type() == CV_8U)
  {
    std::copy(descriptor.begin<uint8_t>(), descriptor.end<uint8_t>(), std::back_inserter(ros_descriptor.udata));
  }
  else if (descriptor.type() == CV_16U)
  {
    std::copy(descriptor.begin<uint16_t>(), descriptor.end<uint16_t>(), std::back_inserter(ros_descriptor.udata));
  }
  else if (descriptor.type() == CV_8S)
  {
    std::copy(descriptor.begin<int8_t>(), descriptor.end<int8_t>(), std::back_inserter(ros_descriptor.idata));
  }
  else if (descriptor.type() == CV_16S)
  {
    std::copy(descriptor.begin<int16_t>(), descriptor.end<int16_t>(), std::back_inserter(ros_descriptor.idata));
  }
  else if (descriptor.type() == CV_32S)
  {
    std::copy(descriptor.begin<int32_t>(), descriptor.end<int32_t>(), std::back_inserter(ros_descriptor.idata));
  }
  else if ((descriptor.type() == CV_32F) ||
      (descriptor.type() == CV_64F))
  {
    // TODO: check that this works.
    std::copy(descriptor.begin<double>(), descriptor.end<double>(), std::back_inserter(ros_descriptor.fdata));
  }
  else
  {
    ROS_ERROR("Unrecognized data type: %d", descriptor.type());
    throw ros::Exception("Unrecognized data type");
  }
  return ros_descriptor;
}

/* Copy the data from a ROS Feature to an OpenCV feature (1 x n matrix).
 */
inline void cvFromRos(const Feature& ros_descriptor, cv::Mat& cv_descriptor, const int type)
{
  if (type == CV_8U)
  {
    assert(ros_descriptor.udata.size() == cv_descriptor.cols);
    for (size_t j = 0; j < ros_descriptor.udata.size(); ++j)
    {
      cv_descriptor.at<uint8_t>(0, j) = (uint8_t)ros_descriptor.udata[j];
    }
  }
  else if (type == CV_16U)
  {
    assert(ros_descriptor.udata.size() == cv_descriptor.cols);
    for (size_t j = 0; j < ros_descriptor.udata.size(); ++j)
    {
      cv_descriptor.at<uint16_t>(0, j) = (uint16_t)ros_descriptor.udata[j];
    }
  }
  else if (type == CV_8S)
  {
    assert(ros_descriptor.idata.size() == cv_descriptor.cols);
    for (size_t j = 0; j < ros_descriptor.idata.size(); ++j)
    {
      cv_descriptor.at<int8_t>(0, j) = (int8_t)ros_descriptor.idata[j];
    }
  }
  else if (type == CV_16S)
  {
    assert(ros_descriptor.idata.size() == cv_descriptor.cols);
    for (size_t j = 0; j < ros_descriptor.idata.size(); ++j)
    {
      cv_descriptor.at<int16_t>(0, j) = (int16_t)ros_descriptor.idata[j];
    }
  }
  else if (type == CV_32S)
  {
    assert(ros_descriptor.idata.size() == cv_descriptor.cols);
    for (size_t j = 0; j < ros_descriptor.idata.size(); ++j)
    {
      cv_descriptor.at<int32_t>(0, j) = (int32_t)ros_descriptor.idata[j];
    }
  }
  else if (type == CV_32F)
  {
    // TODO: check this.
    assert(ros_descriptor.fdata.size() == cv_descriptor.cols);
    for (size_t j = 0; j < ros_descriptor.fdata.size(); ++j)
    {
      cv_descriptor.at<float>(0, j) = ros_descriptor.fdata[j];
    }
  }
  else if (type == CV_64F)
  {
    // TODO: check this.
    assert(ros_descriptor.fdata.size() == cv_descriptor.cols);
    for (size_t j = 0; j < ros_descriptor.fdata.size(); ++j)
    {
      cv_descriptor.at<double>(0, j) = ros_descriptor.fdata[j];
    }
  }
  else
  {
    ROS_ERROR("Unrecognized data type: %d", type);
    throw ros::Exception("Unrecognized data type");
  }
}

inline void rosFromCv(const cv::Mat& cv_descriptors, vector<Feature>& ros_descriptors)
{
  ros_descriptors.clear();
  ros_descriptors.reserve(cv_descriptors.rows);
  for (int i = 0; i < cv_descriptors.rows; ++i)
  {
    const Feature ros_descriptor = rosFromCv(cv_descriptors.row(i));
    ros_descriptors.push_back(ros_descriptor);
  }
}

inline void cvFromRos(const vector<Feature>& ros_descriptors, cv::Mat& cv_descriptors, const int type)
{
  if (ros_descriptors.empty())
  {
    cv_descriptors.create(0, 0, type);
    return;
  }

  int size = std::max(ros_descriptors[0].udata.size(), std::max(ros_descriptors[0].idata.size(), ros_descriptors[0].fdata.size()));
  cv_descriptors.create(ros_descriptors.size(), size, type);

  for (size_t i = 0; i < ros_descriptors.size(); ++i)
  {
    cv::Mat cv_descriptor = cv_descriptors.row(i);
    cvFromRos(ros_descriptors[i], cv_descriptor, type);
  }
}

} /* namespace anj_featurenav */  


#endif /* end of include guard: ALJ_FEATURENAV_JOCKEY_H */
