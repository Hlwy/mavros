/**
 * @brief VisionPoseEstimate plugin
 * @file vision_pose_estimate.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 M.H.Kabir.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <tf2_eigen/tf2_eigen.h>
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <mavros_msgs/VisoParamSet.h>
#include <eigen_conversions/eigen_msg.h>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_extras/VisionEstimatePluginConfig.h>

namespace mavros {
namespace extra_plugins{
/**
 * @brief Vision pose estimate plugin
 *
 * Send pose estimation from various vision estimators
 * to FCU position and attitude estimators.
 *
 */
class VisionPoseEstimatePlugin : public plugin::PluginBase,
	private plugin::TF2ListenerMixin<VisionPoseEstimatePlugin> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VisionPoseEstimatePlugin() : PluginBase(),
		sp_nh("~vision_pose"), roll_offset_(0.0), pitch_offset_(0.0), yaw_offset_(0.0),
		tf_rate(10.0), px4_tf_rotate_(true), send_pose_estimate_(false),
		check_pose_glitch_(true), min_glitch_confidence_(0.0), mid_glitch_confidence_(0.0),
		dist_min_err_thresh_(0.5), dist_mid_err_thresh_(0.5), dist_max_err_thresh_(0.1),
		max_confidence_(100.0), tf_send(false)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		bool tf_listen;

		// tf params
		sp_nh.param("apm_use_delta", apm_use_deltas_, false);
		sp_nh.param("px4_tf_rotate", px4_tf_rotate_, false);
		sp_nh.param("send_pose_estimate", send_pose_estimate_, false);
		sp_nh.param("check_pose_glitch", check_pose_glitch_, false);
		sp_nh.param("dist_min_err_thresh", dist_min_err_thresh_, 0.05);
		sp_nh.param("dist_mid_err_thresh", dist_mid_err_thresh_, 0.05);
		sp_nh.param("dist_max_err_thresh", dist_max_err_thresh_, 0.1);
		sp_nh.param("good_health_thresh", good_health_thresh_, 0.1);
		sp_nh.param("bad_health_thresh", bad_health_thresh_, 1000.0);
		sp_nh.param("max_confidence", max_confidence_, 100.0);
		sp_nh.param("default_confidence", default_confidence_, 50.0);
		sp_nh.param("min_glitch_confidence", min_glitch_confidence_, 1.0);
		sp_nh.param("mid_glitch_confidence", mid_glitch_confidence_, 1.0);
		sp_nh.param("offsets/roll", roll_offset_, 0.0);
		sp_nh.param("offsets/pitch", pitch_offset_, 0.0);
		sp_nh.param("offsets/yaw", yaw_offset_, 0.0);
		sp_nh.param("tf/listen", tf_listen, false);
		sp_nh.param("tf/send", tf_send, false);
		sp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		sp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "vision_estimate");
		sp_nh.param("tf/rate_limit", tf_rate, 10.0);

		ROS_INFO_NAMED("vision_pose", "Vision Pose Offsets (Roll, Pitch, Yaw) --- %.3lf, %.3lf, %.3lf",
			roll_offset_, pitch_offset_, yaw_offset_
		);
		if(send_pose_estimate_){
			debug_pose_pub = sp_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_estimate", 10);
		}
		if (tf_listen) {
			ROS_INFO_STREAM_NAMED("vision_pose", "Listen to vision transform " << tf_frame_id << " -> " << tf_child_frame_id);
			tf2_start("VisionPoseTF", &VisionPoseEstimatePlugin::transform_cb);
		} else {
			vision_sub = sp_nh.subscribe("pose", 10, &VisionPoseEstimatePlugin::vision_cb, this);
			vision_cov_sub = sp_nh.subscribe("pose_cov", 10, &VisionPoseEstimatePlugin::vision_cov_cb, this);
		}
		service = sp_nh.advertiseService("set_param", &VisionPoseEstimatePlugin::param_callback, this);
		// last_pose.setZero();
		_cfg_f = boost::bind(&VisionPoseEstimatePlugin::cfgCallback, this, _1, _2);
		_cfg_server.setCallback(_cfg_f);
	}

	Subscriptions get_subscriptions(){ return { /* Rx disabled */ }; }

private:
	friend class TF2ListenerMixin;
	ros::NodeHandle sp_nh;

	ros::Subscriber vision_sub;
	ros::Subscriber vision_cov_sub;
	ros::Publisher debug_pose_pub;
	ros::ServiceServer service;
	dynamic_reconfigure::Server<mavros_extras::VisionEstimatePluginConfig> _cfg_server;
	dynamic_reconfigure::Server<mavros_extras::VisionEstimatePluginConfig>::CallbackType _cfg_f;

	bool px4_tf_rotate_;
	bool apm_use_deltas_;
	bool send_pose_estimate_;
	bool enable_vision_pose_estimate_mavlink_ = true;
	bool enable_vision_pose_delta_mavlink_ = true;
	bool check_pose_glitch_;
	double dist_min_err_thresh_;
	double dist_mid_err_thresh_;
	double dist_max_err_thresh_;
	double good_health_thresh_;
	double bad_health_thresh_;
	double max_confidence_;
	double default_confidence_;
	double min_glitch_confidence_;
	double mid_glitch_confidence_;
	std::string tf_frame_id;
	std::string tf_child_frame_id;
	double tf_rate;
	ros::Time last_transform_stamp;
	Eigen::Affine3d last_pose;
	Eigen::Vector3d prev_position;
	Eigen::Matrix3d prev_rotation;
	Eigen::Matrix3d prev_rotation_inverse;
	bool last_pose_recvd = false;
	double roll_offset_;
	double pitch_offset_;
	double yaw_offset_;
	bool tf_send;
	mavlink::common::msg::VISION_POSITION_ESTIMATE prev_vpp;

	bool param_callback(mavros_msgs::VisoParamSet::Request& request, mavros_msgs::VisoParamSet::Response& response){
		if( (request.param_id.compare("max_confidence")) == 0 ){
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", request.param_id.c_str(), request.value);
			this->max_confidence_ = (double) request.value;
			response.success = true;
		} else if( (request.param_id.compare("default_confidence")) == 0 ){
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", request.param_id.c_str(), request.value);
			this->default_confidence_ = (double) request.value;
			response.success = true;
		} else if( (request.param_id.compare("min_glitch_confidence")) == 0 ){
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", request.param_id.c_str(), request.value);
			this->min_glitch_confidence_ = (double) request.value;
			response.success = true;
		} else if( (request.param_id.compare("mid_glitch_confidence")) == 0 ){
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", request.param_id.c_str(), request.value);
			this->mid_glitch_confidence_ = (double) request.value;
			response.success = true;
		} else if( (request.param_id.compare("good_health_thresh")) == 0 ){
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", request.param_id.c_str(), request.value);
			this->good_health_thresh_ = (double) request.value;
			response.success = true;
		} else if( (request.param_id.compare("bad_health_thresh")) == 0 ){
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", request.param_id.c_str(), request.value);
			this->bad_health_thresh_ = (double) request.value;
			response.success = true;
		} else if( (request.param_id.compare("dist_min_err_thresh")) == 0 ){
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", request.param_id.c_str(), request.value);
			this->dist_min_err_thresh_ = (double) request.value;
			response.success = true;
		} else if( (request.param_id.compare("dist_mid_err_thresh")) == 0 ){
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", request.param_id.c_str(), request.value);
			this->dist_mid_err_thresh_ = (double) request.value;
			response.success = true;
		} else if( (request.param_id.compare("dist_max_err_thresh")) == 0 ){
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", request.param_id.c_str(), request.value);
			this->dist_max_err_thresh_ = (double) request.value;
			response.success = true;
		} else if( (request.param_id.compare("good_health_thresh")) == 0 ){
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", request.param_id.c_str(), request.value);
			this->good_health_thresh_ = (double) request.value;
			response.success = true;
		} else if( (request.param_id.compare("bad_health_thresh")) == 0 ){
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", request.param_id.c_str(), request.value);
			this->bad_health_thresh_ = (double) request.value;
			response.success = true;
		} else if( (request.param_id.compare("roll_offset")) == 0 ){
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", request.param_id.c_str(), request.value);
			double tmpAngle = (double) request.value;
			this->roll_offset_ = (double) tmpAngle/180.0*M_PI;
			response.success = true;
		} else if( (request.param_id.compare("pitch_offset")) == 0 ){
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", request.param_id.c_str(), request.value);
			double tmpAngle = (double) request.value;
			this->pitch_offset_ = (double) tmpAngle/180.0*M_PI;
			response.success = true;
		} else if( (request.param_id.compare("yaw_offset")) == 0 ){
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", request.param_id.c_str(), request.value);
			double tmpAngle = (double) request.value;
			this->yaw_offset_ = (double) tmpAngle/180.0*M_PI;
			response.success = true;
		} else if( (request.param_id.compare("send_vision_pose_estimate")) == 0 ){
			double tmpFlag = (double) request.value;
			if(tmpFlag == 0) this->enable_vision_pose_estimate_mavlink_ = false;
			else if(tmpFlag == 1) this->enable_vision_pose_estimate_mavlink_ = true;
			else this->enable_vision_pose_estimate_mavlink_ = false;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %s", request.param_id.c_str(), (double) request.value == 1 ? "true" : "false");
			response.success = true;
		} else if( (request.param_id.compare("send_vision_pose_delta")) == 0 ){
			double tmpFlag = (double) request.value;
			if(tmpFlag == 0) this->enable_vision_pose_delta_mavlink_ = false;
			else if(tmpFlag == 1) this->enable_vision_pose_delta_mavlink_ = true;
			else this->enable_vision_pose_delta_mavlink_ = false;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %s", request.param_id.c_str(), (double) request.value == 1 ? "true" : "false");
			response.success = true;
		} else{
			ROS_INFO_NAMED("vision_pose", "Requested parameter \'%s\' doesn't match any known parameters", request.param_id.c_str());
			response.success = false;
		}
		return true;
	}

	void cfgCallback(mavros_extras::VisionEstimatePluginConfig &config, uint32_t level){
		if(config.check_pose_glitch != check_pose_glitch_){
			check_pose_glitch_ = config.check_pose_glitch;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %s", "check_pose_glitch_", check_pose_glitch_ ? "true" : "false");
		}
		if(config.publish_pose_estimate != send_pose_estimate_){
			send_pose_estimate_ = config.publish_pose_estimate;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %s", "send_pose_estimate_", send_pose_estimate_ ? "true" : "false");
		}
		if(config.send_vision_pose_delta != enable_vision_pose_delta_mavlink_){
			enable_vision_pose_delta_mavlink_ = config.send_vision_pose_delta;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %s", "enable_vision_pose_delta_mavlink_", enable_vision_pose_delta_mavlink_ ? "true" : "false");
		}
		if(config.send_vision_pose_estimate != enable_vision_pose_estimate_mavlink_){
			enable_vision_pose_estimate_mavlink_ = config.send_vision_pose_estimate;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %s", "enable_vision_pose_estimate_mavlink_", enable_vision_pose_estimate_mavlink_ ? "true" : "false");
		}
		if(config.good_health_thresh != good_health_thresh_){
			good_health_thresh_ = config.good_health_thresh;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", "good_health_thresh_", good_health_thresh_);
		}
		if(config.bad_health_thresh != bad_health_thresh_){
			bad_health_thresh_ = config.bad_health_thresh;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", "bad_health_thresh_", bad_health_thresh_);
		}
		if(config.max_confidence != max_confidence_){
			max_confidence_ = config.max_confidence;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", "max_confidence_", max_confidence_);
		}
		if(config.default_confidence != default_confidence_){
			default_confidence_ = config.default_confidence;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", "default_confidence_", default_confidence_);
		}
		if(config.min_glitch_confidence != min_glitch_confidence_){
			min_glitch_confidence_ = config.min_glitch_confidence;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", "min_glitch_confidence_", min_glitch_confidence_);
		}
		if(config.mid_glitch_confidence != mid_glitch_confidence_){
			mid_glitch_confidence_ = config.mid_glitch_confidence;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", "mid_glitch_confidence_", mid_glitch_confidence_);
		}
		if(config.dist_min_err_thresh != dist_min_err_thresh_){
			dist_min_err_thresh_ = config.dist_min_err_thresh;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", "dist_min_err_thresh_", dist_min_err_thresh_);
		}
		if(config.dist_mid_err_thresh != dist_mid_err_thresh_){
			dist_mid_err_thresh_ = config.dist_mid_err_thresh;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", "dist_mid_err_thresh_", dist_mid_err_thresh_);
		}
		if(config.dist_max_err_thresh != dist_max_err_thresh_){
			dist_max_err_thresh_ = config.dist_max_err_thresh;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", "dist_max_err_thresh_", dist_max_err_thresh_);
		}

		if(config.roll_offset != roll_offset_){
			roll_offset_ = config.roll_offset;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", "roll_offset_", roll_offset_);
		}
		if(config.pitch_offset != pitch_offset_){
			pitch_offset_ = config.pitch_offset;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", "pitch_offset_", pitch_offset_);
		}
		if(config.yaw_offset != yaw_offset_){
			yaw_offset_ = config.yaw_offset;
			ROS_INFO_NAMED("vision_pose", "Setting parameter \'%s\' to %.1f", "yaw_offset_", yaw_offset_);
		}
	}

	// void publish_tf(const ros::Time &stamp, const Eigen::Affine3d &tr)
	void publish_tf(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		if (tf_send) {
			geometry_msgs::TransformStamped transform;
			transform.header.stamp = msg->header.stamp;
			transform.header.frame_id = tf_frame_id;
			transform.child_frame_id = tf_child_frame_id;
			transform.transform.translation.x = msg->pose.position.x;
			transform.transform.translation.y = msg->pose.position.y;
			transform.transform.translation.z = msg->pose.position.z;
			transform.transform.rotation = msg->pose.orientation;
			m_uas->tf2_broadcaster.sendTransform(transform);
		}
	}
	/* -*- low-level send -*- */

	/**
	 * @brief Send vision estimate transform to FCU position controller
	 */
	void send_vision_estimate(const ros::Time &stamp, const Eigen::Affine3d &tr, const geometry_msgs::PoseWithCovariance::_covariance_type &cov)
	{
		/**
		 * @warning Issue #60.
		 * This now affects pose callbacks too.
		 */
		if (last_transform_stamp == stamp) {
			ROS_DEBUG_THROTTLE_NAMED(10, "vision_pose", "Vision: Same transform as last one, dropped.");
			return;
		}

		if(!apm_use_deltas_){
			Eigen::Vector3d position;
			Eigen::Vector3d rpy;
			// Eigen::Quaterniond quats = Eigen::Quaterniond(tr.rotation());
			if(px4_tf_rotate_){
				position = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
				rpy = ftf::quaternion_to_rpy(ftf::transform_orientation_enu_ned(ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation()))));
			} else{
				position = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
				rpy = ftf::quaternion_to_rpy(ftf::transform_orientation_enu_ned(Eigen::Quaterniond(tr.rotation())));
			}
			ROS_DEBUG_NAMED("vision_pose", "Vision Pose Un-Corrected R, P, Y --- %.5lf, %.5lf, %.5lf",
				rpy.x(), rpy.y(), rpy.z()
			);
			double corrected_roll = rpy.x() - roll_offset_;
			double corrected_pitch = rpy.y() - pitch_offset_;
			double corrected_yaw = rpy.z() - yaw_offset_;
			ROS_DEBUG_NAMED("vision_pose", "Vision Pose Corrected R, P, Y --- %.5lf, %.5lf, %.5lf",
				corrected_roll, corrected_pitch, corrected_yaw
			);
			Eigen::Matrix3d cur_rotation;
			cur_rotation = Eigen::AngleAxisd(corrected_roll, Eigen::Vector3d::UnitX())
						* Eigen::AngleAxisd(corrected_pitch, Eigen::Vector3d::UnitY())
						* Eigen::AngleAxisd(corrected_yaw, Eigen::Vector3d::UnitZ());


			mavlink::common::msg::VISION_POSITION_ESTIMATE vp{};
			vp.usec = (uint64_t) (stamp.toNSec() / 1000);
			vp.x = position.y();
			vp.y = -position.x();
			vp.z = position.z();
			vp.roll = rpy.x();
			vp.pitch = rpy.y();
			vp.yaw = rpy.z();

			// auto cov_ned = ftf::transform_frame_enu_ned(cov);
			// ftf::EigenMapConstCovariance6d cov_map(cov_ned.data());
			// auto urt_view = Eigen::Matrix<double, 6, 6>(cov_map.triangularView<Eigen::Upper>());
			// // ROS_DEBUG_STREAM_NAMED("vision_pose", "Vision: Covariance URT: " << std::endl << urt_view);
			// // just the URT of the 6x6 Pose Covariance Matrix, given
			// // that the matrix is symmetric
			// ftf::covariance_urt_to_mavlink(cov_map, vp.covariance);
			ROS_DEBUG_NAMED("vision_pose", "Vision Pose (Time, X, Y, Z, R, P, Y) --- %d, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f",
				(int) vp.usec, vp.x, vp.y, vp.z, vp.roll, vp.pitch, vp.yaw
			);
			UAS_FCU(m_uas)->send_message_ignore_drop(vp);
			ROS_DEBUG_STREAM_NAMED("vision_pose", "VISION_POSITION_ESTIMATE Msg sent! " << std::endl);
		} else{
			Eigen::Vector3d position = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
			// Eigen::Vector3d rpy = ftf::quaternion_to_rpy(ftf::transform_orientation_enu_ned(Eigen::Quaterniond(tr.rotation())));
			Eigen::Vector3d rpy = ftf::quaternion_to_rpy(
							  ftf::transform_orientation_enu_ned(
							  ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())
						  	))
			);
			Eigen::Matrix3d cur_rotation;
			cur_rotation = Eigen::AngleAxisd(rpy.x() + roll_offset_, Eigen::Vector3d::UnitX())
						* Eigen::AngleAxisd(rpy.y() + pitch_offset_, Eigen::Vector3d::UnitY())
						* Eigen::AngleAxisd(rpy.z() + yaw_offset_, Eigen::Vector3d::UnitZ());
			if(!last_pose_recvd){
				prev_position = position;
				prev_rotation = cur_rotation;
				prev_rotation_inverse = prev_rotation.inverse();
				last_pose_recvd = true;
				ROS_DEBUG_THROTTLE_NAMED(10, "vision_pose", "Vision: Same Pose as last one, dropped.");
				return;
			}

			double dx = position.x() - prev_position.x();
			double dy = position.y() - prev_position.y();
			double distErrMetric = std::sqrt(dx*dx + dy*dy);
			// ROS_DEBUG_THROTTLE_NAMED(5, "vision_pose", "Pose Error = %.4lf -- dX, dY = %.4lf, %.4lf",distErrMetric, dx, dy);

			Eigen::Vector3d delta_pos = position - prev_position;
			delta_pos = prev_rotation_inverse * delta_pos;
			Eigen::Matrix3d delta_rot = prev_rotation_inverse * cur_rotation;
			Eigen::Vector3d delta_rpy = ftf::quaternion_to_rpy(Eigen::Quaterniond(delta_rot));

			prev_position = position;
			prev_rotation = cur_rotation;
			prev_rotation_inverse = prev_rotation.inverse();
			// ROS_DEBUG_STREAM_NAMED("vision_pose", "Vision: Delta Position: " << std::endl << delta_pos);
			// ROS_DEBUG_STREAM_NAMED("vision_pose", "Vision: Delta RPY: " << std::endl << delta_rpy);

			auto cov_ned = ftf::transform_frame_enu_ned(cov);
			ftf::EigenMapConstCovariance6d cov_map(cov_ned.data());
			auto urt_view = Eigen::Matrix<double, 6, 6>(cov_map.triangularView<Eigen::Upper>());
			// ROS_DEBUG_STREAM_NAMED("vision_pose", "Vision: Covariance URT: " << std::endl << urt_view(0,0));

			double confidence = 0.0;
			double cov_health_metric = urt_view(0,0);
			if((cov_health_metric <= good_health_thresh_) && (cov_health_metric != 0.0) ){
				confidence = max_confidence_;
				// ROS_DEBUG_NAMED("vision_pose", "Highest Vision Pose Estimate Health (%.1lf) ---- Confidence Metric (%.5lf) <= Good Health Threshold (%.5lf)", confidence, cov_health_metric, good_health_thresh);
			} else if((cov_health_metric >= bad_health_thresh_) && (cov_health_metric != 0.0) ){
				confidence = 0.0;
				// ROS_DEBUG_NAMED("vision_pose", "Lowest Vision Pose Estimate Health (%.1lf) ---- Confidence Metric (%.5lf) >= Bad Health Threshold (%.5lf)", confidence, cov_health_metric, bad_health_thresh);
			} else if(cov_health_metric == 0.0){
				confidence = max_confidence_;
				ROS_DEBUG_NAMED("vision_pose", "Null Vision Pose Estimate Health (%.1lf) ---- Confidence Metric (%.5lf) == 0", confidence, cov_health_metric);
			} else{
				confidence = default_confidence_;
				ROS_DEBUG_NAMED("vision_pose", "Default Vision Pose Estimate Health (%.1lf) ---- Confidence Metric (%.5lf)", confidence, cov_health_metric);
			}

			if( (confidence > 0.0) && (check_pose_glitch_) ){
				if(std::fabs(dx) >= dist_max_err_thresh_){
					ROS_DEBUG_NAMED("vision_pose", "Vision Pose Delta Major Glitch --> Estimate Health = 0.0");
					ROS_DEBUG_NAMED("vision_pose", " --- Pose Error (%.5lf) -- dX, dY = %.4lf, %.4lf", distErrMetric, dx, dy);
					confidence = 0.0;
				} else if( (std::fabs(dx) >= dist_mid_err_thresh_) && (std::fabs(dx) < dist_max_err_thresh_) ){
					ROS_DEBUG_NAMED("vision_pose", "Vision Pose Delta Medium Glitch --> Estimate Health = %.1lf", mid_glitch_confidence_);
					ROS_DEBUG_NAMED("vision_pose", " --- Pose Error (%.5lf) -- dX, dY = %.4lf, %.4lf", distErrMetric, dx, dy);
					confidence = mid_glitch_confidence_;
				} else if( (std::fabs(dx) >= dist_min_err_thresh_) && (std::fabs(dx) < dist_mid_err_thresh_) ){
					ROS_DEBUG_NAMED("vision_pose", "Vision Pose Delta Minor Glitch --> Estimate Health = %.1lf", min_glitch_confidence_);
					ROS_DEBUG_NAMED("vision_pose", " --- Pose Error (%.5lf) -- dX, dY = %.4lf, %.4lf", distErrMetric, dx, dy);
					confidence = min_glitch_confidence_;
				}
			}
			// if( (confidence > 0.0) && (check_pose_glitch) ){
			// 	if(distErrMetric >= dist_max_err_thresh){
			// 		ROS_DEBUG_NAMED("vision_pose", "Vision Pose Major Glitch --> Setting Estimate Health to (0.0)");
			// 		ROS_DEBUG_NAMED("vision_pose", " --- Pose Error (%.5lf) -- dX, dY = %.4lf, %.4lf", distErrMetric, dx, dy);
			// 		confidence = 0.0;
			// 	} else if( (distErrMetric >= dist_err_thresh) && (distErrMetric < dist_max_err_thresh) ){
			// 		ROS_DEBUG_NAMED("vision_pose", "Vision Pose Minor Glitch --> Setting  Estimate Health to (%.1lf)", glitch_confidence);
			// 		ROS_DEBUG_NAMED("vision_pose", " --- Pose Error (%.5lf) -- dX, dY = %.4lf, %.4lf", distErrMetric, dx, dy);
			// 		confidence = glitch_confidence;
			// 	}
			// }

			// mavlink::common::msg::VISION_POSITION_ESTIMATE vp{};
			mavlink::ardupilotmega::msg::VISION_POSITION_DELTA vp{};
			vp.time_usec = stamp.toNSec() / 1000;
			vp.time_delta_usec = vp.time_usec - (last_transform_stamp.toNSec() / 1000);
			vp.position_delta[0] = delta_pos.x();
			vp.position_delta[1] = delta_pos.y();
			vp.position_delta[2] = delta_pos.z();
			vp.angle_delta[0] = delta_rpy.x();
			vp.angle_delta[1] = delta_rpy.y();
			vp.angle_delta[2] = delta_rpy.z();
			vp.confidence = confidence;

			// [[[end]]] (checksum: 2048daf411780847e77f08fe5a0b9dd3)

			// just the URT of the 6x6 Pose Covariance Matrix, given
			// that the matrix is symmetric
			if(enable_vision_pose_delta_mavlink_) UAS_FCU(m_uas)->send_message_ignore_drop(vp);
			// ROS_DEBUG_STREAM_NAMED("vision_pose", "VISION_POSITION_DELTA Msg sent! " << std::endl);

			if(enable_vision_pose_estimate_mavlink_){
				mavlink::common::msg::VISION_POSITION_ESTIMATE vpp{};

				vpp.usec = (uint64_t) (stamp.toNSec() / 1000);
				vpp.x = position.x();
				vpp.y = position.y();
				vpp.z = position.z();
				vpp.roll = rpy.x();
				vpp.pitch = rpy.y();
				vpp.yaw = rpy.z();
				ftf::covariance_urt_to_mavlink(cov_map, vpp.covariance);


				if(std::fabs(dx) >= dist_max_err_thresh_){
					ROS_WARN_NAMED("vision_pose", "Vision Pose Estimate Major Glitch Detected --> Using previous pose");
					ROS_WARN_NAMED("vision_pose", " --- Glitch Pose Error (%.5lf) -- dX, dY = %.4lf, %.4lf", distErrMetric, dx, dy);
					UAS_FCU(m_uas)->send_message_ignore_drop(prev_vpp);
					ROS_DEBUG_NAMED("vision_pose", "VISION_POSITION_ESTIMATE Msg sent! ");
				} else if( (std::fabs(dx) >= dist_mid_err_thresh_) && (std::fabs(dx) < dist_max_err_thresh_) ){
					ROS_WARN_NAMED("vision_pose", "Vision Pose Estimate Medium Glitch --> Using previous pose");
					ROS_WARN_NAMED("vision_pose", " --- Glitch Pose Error (%.5lf) -- dX, dY = %.4lf, %.4lf", distErrMetric, dx, dy);
					UAS_FCU(m_uas)->send_message_ignore_drop(prev_vpp);
					ROS_DEBUG_NAMED("vision_pose", "VISION_POSITION_ESTIMATE Msg sent! ");
				} else{
					ROS_DEBUG_NAMED("vision_pose", "Vision Pose Estimate (Time, X, Y, Z, R, P, Y) --- %d, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f",
						(int) vpp.usec, vpp.x, vpp.y, vpp.z, vpp.roll, vpp.pitch, vpp.yaw
					);
					UAS_FCU(m_uas)->send_message_ignore_drop(vpp);
					prev_vpp = vpp;
					ROS_DEBUG_NAMED("vision_pose", "VISION_POSITION_ESTIMATE Msg sent! ");
				}
			}

			if(send_pose_estimate_){
				// geometry_msgs::Quaternion quat;
				Eigen::Quaterniond quat {};
				geometry_msgs::PoseWithCovarianceStamped pMsg;

				// ROS_DEBUG_STREAM_NAMED("vision_pose", "Vision: Covariance URT: " << std::endl << urt_view);

				// header
				pMsg.header.stamp = stamp;
				pMsg.header.frame_id = tf_frame_id;
				// pose
				pMsg.pose.pose.position.x = position.x();
				pMsg.pose.pose.position.y = position.y();
				pMsg.pose.pose.position.z = position.z();
				// quat = Eigen::Quaterniond(cur_rotation);
				quat = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())
				));
				// quat = Eigen::Quaterniond(
				// 	ftf::transform_orientation_enu_ned(
				// 		ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())
				// )));

				tf::quaternionEigenToMsg(quat, pMsg.pose.pose.orientation);
				// quat = tf2::toMsg(ftf::quaternion_from_rpy(rpy.x(), rpy.y(), rpy.z()));
				pMsg.pose.covariance[0] = urt_view(0,0);	// x
				pMsg.pose.covariance[7] = urt_view(1,1);	// y
				pMsg.pose.covariance[14] = urt_view(2,2);
				debug_pose_pub.publish(pMsg);
			}
			last_transform_stamp = stamp;
		}
	}

	/* common TF listener moved to mixin */
	void transform_cb(const geometry_msgs::TransformStamped &transform){
		Eigen::Affine3d tr;
		tf::transformMsgToEigen(transform.transform, tr);
		ftf::Covariance6d cov {};	// zero initialized
		send_vision_estimate(transform.header.stamp, tr, cov);
	}

	void vision_cb(const geometry_msgs::PoseStamped::ConstPtr &req){
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose, tr);
		ftf::Covariance6d cov {};	// zero initialized
		publish_tf(req);
		send_vision_estimate(req->header.stamp, tr, cov);
	}

	void vision_cov_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &req){
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose.pose, tr);
		send_vision_estimate(req->header.stamp, tr, req->pose.covariance);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VisionPoseEstimatePlugin, mavros::plugin::PluginBase)
