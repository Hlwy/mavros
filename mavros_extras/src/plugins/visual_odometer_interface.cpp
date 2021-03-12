#include <mavros/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <mavros_msgs/Mavlink.h>
#include <mavconn/mavlink_dialect.h>
#include <mavros_extras/VisualOdometerInterfacePluginConfig.h>

using ::mavlink::mavlink_message_t;

typedef enum VisionPoseGlitchLevel{
	NONE = 0,
	MINOR = 1,
	MODERATE = 2,
	SERIOUS = 3,
	CRITICAL = 4,
}VisionPoseGlitchLevel;
typedef enum GlitchDetectionMetricType{
	X_AXIS_ONLY = 0,
	Y_AXIS_ONLY = 1,
	Z_AXIS_ONLY = 2,
	EUCLIDEAN_DISTANCE_2D = 3,
	EUCLIDEAN_DISTANCE_3D = 4,
}GlitchDetectionMetricType;

namespace mavros {
namespace extra_plugins{

/**
if(publish_to_gcs_){
	mavlink_message_t vpMsg{};
	mavlink::MsgMap map(vpMsg);
	// mavros_msgs::Mavlink* mavMsg;
	auto mavMsg = boost::make_shared<mavros_msgs::Mavlink>();
	vp.serialize(map);
	// mavlink::mavlink_finalize_message(vpMsg, UAS_FCU(m_uas)->get_system_id(), UAS_FCU(m_uas)->get_component_id(), vp.MIN_LENGTH, vp.LENGTH, vp.CRC_EXTRA);
	mavMsg->header.stamp = stamp;
	mavros_msgs::mavlink::convert(vpMsg, *mavMsg);
	ROS_DEBUG_NAMED("vision_pose", "Sent VISION_POSITION_DELTA Msg to GCS Bridge.");
}
*/

/**
 * @brief Vision pose estimate plugin
 *
 * Send pose estimation from various vision estimators
 * to FCU position and attitude estimators.
 *
 */
class VisualOdometerInterfacePlugin : public plugin::PluginBase,
	private plugin::TF2ListenerMixin<VisualOdometerInterfacePlugin> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VisualOdometerInterfacePlugin() : PluginBase(), sp_nh("~visual_odometer"){}

	Subscriptions get_subscriptions(){ return { /* Rx disabled */ }; }

	void initialize(UAS &uas_){
		PluginBase::initialize(uas_);

		// ROS Param Configuration Defaults
		sp_nh.param("offsets/roll", 			roll_offset_, 			0.0);
		sp_nh.param("offsets/pitch", 			pitch_offset_, 		0.0);
		sp_nh.param("offsets/yaw", 			yaw_offset_, 			0.0);
		sp_nh.param("check_pose_glitch", 		check_pose_glitch_, 	false);
		sp_nh.param("dist_min_err_thresh", 	glitch_dist_thresh_min_err_, 	0.05);
		sp_nh.param("dist_mid_err_thresh", 	glitch_dist_thresh_mid_err_, 	0.05);
		sp_nh.param("dist_max_err_thresh", 	glitch_dist_thresh_max_err_, 	0.1);
		sp_nh.param("good_health_thresh", 		good_health_thresh_, 	0.1);
		sp_nh.param("bad_health_thresh", 		bad_health_thresh_, 	1000.0);
		sp_nh.param("max_confidence", 		max_confidence_, 		100.0);
		sp_nh.param("default_confidence", 		default_confidence_, 	50.0);
		sp_nh.param("min_glitch_confidence", 	min_glitch_confidence_, 	1.0);
		sp_nh.param("mid_glitch_confidence", 	mid_glitch_confidence_, 	1.0);
		sp_nh.param("publish_debug_pose_estimate", publish_debug_pose_estimate_, false);

		bool tf_listen;
		sp_nh.param("tf/send", tf_send, false);
		sp_nh.param("tf/listen", tf_listen, false);
		sp_nh.param("tf/rate_limit", tf_rate, 10.0);
		sp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		sp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "vision_estimate");
		if(tf_listen){
			ROS_INFO_STREAM_NAMED("visual_odometer", "Listen to vision transform " << tf_frame_id << " -> " << tf_child_frame_id);
			tf2_start("VisionPoseTF", &VisualOdometerInterfacePlugin::transform_cb);
		} else{
			vision_pose_sub = sp_nh.subscribe("pose", 10, &VisualOdometerInterfacePlugin::vision_pose_cb, this);
			vision_pose_cov_sub = sp_nh.subscribe("pose_cov", 10, &VisualOdometerInterfacePlugin::vision_pose_cov_cb, this);
			vision_speed_sub = sp_nh.subscribe("speed_twist", 10, &VisualOdometerInterfacePlugin::twist_cb, this);
			vision_speed_cov_sub = sp_nh.subscribe("speed_twist_cov", 10, &VisualOdometerInterfacePlugin::twist_cov_cb, this);
		}

		// Construct other ROS Object required
		debug_pose_pub = sp_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_estimate_debug", 10);
		_cfg_f = boost::bind(&VisualOdometerInterfacePlugin::cfgCallback, this, _1, _2);
		_cfg_server.setCallback(_cfg_f);
	}

private:
	friend class TF2ListenerMixin;
	ros::NodeHandle sp_nh;
	ros::Publisher debug_pose_pub;
	ros::Subscriber vision_pose_sub;
	ros::Subscriber vision_speed_sub;
	ros::Subscriber vision_pose_cov_sub;
	ros::Subscriber vision_speed_cov_sub;
	dynamic_reconfigure::Server<mavros_extras::VisualOdometerInterfacePluginConfig> _cfg_server;
	dynamic_reconfigure::Server<mavros_extras::VisualOdometerInterfacePluginConfig>::CallbackType _cfg_f;

	// tf- related
	bool tf_send 		 					= false;
	double tf_rate 	 					= 10.0;
	std::string tf_frame_id 					= "odom";
	std::string tf_child_frame_id 			= "base_link";

	bool publish_debug_pose_estimate_ 			= false;
	bool enable_vision_pose_delta_mavlink_ 		= false;
	bool enable_vision_velocities_mavlink_ 		= true;
	bool enable_vision_pose_estimate_mavlink_ 	= true;
	bool test_position_offset_corrections_ 		= false;
	bool test_velocity_offset_corrections_ 		= false;

	bool check_pose_glitch_					= true;
	double glitch_dist_thresh_min_err_			= 0.05;
	double glitch_dist_thresh_mid_err_			= 0.1;
	double glitch_dist_thresh_max_err_			= 0.5;
	GlitchDetectionMetricType glitchDetectMeth_  = X_AXIS_ONLY;
	std::string glitchDetectMethStr_ 			= "X_AXIS_ONLY";

	// Parameters specific to VISION_POSITION_DELTA mavlink messaging
	double good_health_thresh_	= 0.001;
	double bad_health_thresh_	= 1000.0;
	double max_confidence_		= 100.0;
	double default_confidence_	= 50.0;
	double min_glitch_confidence_	= 80.0;
	double mid_glitch_confidence_	= 60.0;

	// Pose Offsets [m or rad]
	double roll_offset_	 = 0.0;
	double pitch_offset_ = 0.0;
	double yaw_offset_	 = 0.0;
	// Pose + Velocity Tracking Storage Variables
	bool last_pose_recvd = false;
	ros::Time prev_pose_stamp_;
	ros::Time cur_speeds_stamp_;
	ros::Time prev_speeds_stamp_;
	// Pose-specific variables
	Eigen::Vector3d prev_rpy;
	Eigen::Vector3d prev_position;
	Eigen::Matrix3d prev_rotation;
	Eigen::Matrix3d prev_rotation_inverse;
	// Velocity-specific variables
	Eigen::Vector3d cur_speeds_;
	Eigen::Vector3d prev_speeds_;
	ftf::Covariance3d cur_speeds_cov_;
	ftf::Covariance3d prev_speeds_cov_;
	// Latest Vision Velocity data regardless of stability
	ros::Time latest_velocities_stamp_;
	Eigen::Vector3d latest_velocities_;
	ftf::Covariance3d latest_velocities_cov_;

	// Mavlink message storage containers
	mavlink::common::msg::VISION_SPEED_ESTIMATE prev_vse;
	mavlink::common::msg::VISION_POSITION_ESTIMATE prev_vpp;
	// Debugging
	bool dbg_mavlink_					= false;
	bool dbg_deltas_					= false;
	bool dbg_velocities_				= false;
	bool dbg_positions_					= false;
	bool dbg_covariances_				= false;

	/**
	* @brief Begin Function declarations
	*/
	void cfgCallback(mavros_extras::VisualOdometerInterfacePluginConfig &config, uint32_t level){
		if(config.send_vision_velocities != enable_vision_velocities_mavlink_){
			enable_vision_velocities_mavlink_ = config.send_vision_velocities;
			ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %s", "enable_vision_velocities_mavlink_", enable_vision_velocities_mavlink_ ? "true" : "false");
		}
		if(config.send_vision_pose_estimate != enable_vision_pose_estimate_mavlink_){
			enable_vision_pose_estimate_mavlink_ = config.send_vision_pose_estimate;
			ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %s", "enable_vision_pose_estimate_mavlink_", enable_vision_pose_estimate_mavlink_ ? "true" : "false");
		}
		if(config.send_vision_pose_delta != enable_vision_pose_delta_mavlink_){
			enable_vision_pose_delta_mavlink_ = config.send_vision_pose_delta;
			ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %s", "enable_vision_pose_delta_mavlink_", enable_vision_pose_delta_mavlink_ ? "true" : "false");
		}
		if(config.publish_debug_pose_estimate != publish_debug_pose_estimate_){
			publish_debug_pose_estimate_ = config.publish_debug_pose_estimate;
			ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %s", "publish_debug_pose_estimate_", publish_debug_pose_estimate_ ? "true" : "false");
		}
		if(config.viso_frame_id != tf_frame_id){
			tf_frame_id = config.viso_frame_id;
			ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to \'%s\'", "tf_frame_id", tf_frame_id.c_str());
		}
		if(config.viso_child_frame_id != tf_child_frame_id){
			tf_child_frame_id = config.viso_child_frame_id;
			ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to \'%s\'", "tf_child_frame_id", tf_child_frame_id.c_str());
		}

		{ /** Pose-Glitch Detection */
			if(config.do_pose_glitch_checking != check_pose_glitch_){
				check_pose_glitch_ = config.do_pose_glitch_checking;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %s", "check_pose_glitch_", check_pose_glitch_ ? "true" : "false");
			}
			if(config.pose_glitch_detect_method != glitchDetectMeth_){
				std::string newGlitchDetectMethStr_;
				if(config.pose_glitch_detect_method == X_AXIS_ONLY) newGlitchDetectMethStr_ = "X_AXIS_ONLY";
				else if(config.pose_glitch_detect_method == Y_AXIS_ONLY) newGlitchDetectMethStr_ = "Y_AXIS_ONLY";
				else if(config.pose_glitch_detect_method == Z_AXIS_ONLY) newGlitchDetectMethStr_ = "Z_AXIS_ONLY";
				else if(config.pose_glitch_detect_method == EUCLIDEAN_DISTANCE_2D) newGlitchDetectMethStr_ = "2D Euclidean Distance";
				else if(config.pose_glitch_detect_method == EUCLIDEAN_DISTANCE_3D) newGlitchDetectMethStr_ = "3D Euclidean Distance";
				ROS_INFO_NAMED("visual_odometer", "Switching Metric used for detecting pose-glitch occurances from \'%s\' ---> \'%s\'.", glitchDetectMethStr_.c_str(), newGlitchDetectMethStr_.c_str());
				glitchDetectMethStr_ = newGlitchDetectMethStr_;
				glitchDetectMeth_ = (GlitchDetectionMetricType) config.pose_glitch_detect_method;
			}
			if(config.glitch_dist_thresh_min_err != glitch_dist_thresh_min_err_){
				glitch_dist_thresh_min_err_ = config.glitch_dist_thresh_min_err;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %.1f", "glitch_dist_thresh_min_err_", glitch_dist_thresh_min_err_);
			}
			if(config.glitch_dist_thresh_mid_err != glitch_dist_thresh_mid_err_){
				glitch_dist_thresh_mid_err_ = config.glitch_dist_thresh_mid_err;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %.1f", "glitch_dist_thresh_mid_err_", glitch_dist_thresh_mid_err_);
			}
			if(config.glitch_dist_thresh_max_err != glitch_dist_thresh_max_err_){
			glitch_dist_thresh_max_err_ = config.glitch_dist_thresh_max_err;
			ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %.1f", "glitch_dist_thresh_max_err_", glitch_dist_thresh_max_err_);
		}
		}
		{ /** VISION_POSITION_DELTA - Specific */
			if(config.default_confidence != default_confidence_){
				default_confidence_ = config.default_confidence;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %.1f", "default_confidence_", default_confidence_);
			}
			if(config.max_confidence != max_confidence_){
				max_confidence_ = config.max_confidence;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %.1f", "max_confidence_", max_confidence_);
			}
			if(config.good_health_thresh != good_health_thresh_){
				good_health_thresh_ = config.good_health_thresh;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %.1f", "good_health_thresh_", good_health_thresh_);
			}
			if(config.bad_health_thresh != bad_health_thresh_){
				bad_health_thresh_ = config.bad_health_thresh;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %.1f", "bad_health_thresh_", bad_health_thresh_);
			}
			if(config.min_glitch_confidence != min_glitch_confidence_){
				min_glitch_confidence_ = config.min_glitch_confidence;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %.1f", "min_glitch_confidence_", min_glitch_confidence_);
			}
			if(config.mid_glitch_confidence != mid_glitch_confidence_){
				mid_glitch_confidence_ = config.mid_glitch_confidence;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %.1f", "mid_glitch_confidence_", mid_glitch_confidence_);
			}
		}
		{ /** Misc */
			if(config.roll_offset != roll_offset_){
				roll_offset_ = config.roll_offset;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %.1f deg (%.3lf rad)", "roll_offset_", roll_offset_, roll_offset_ * (M_PI/180.0));
			}
			if(config.pitch_offset != pitch_offset_){
				pitch_offset_ = config.pitch_offset;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %.1f deg (%.3lf rad)", "pitch_offset_", pitch_offset_, pitch_offset_ * (M_PI/180.0));
			}
			if(config.yaw_offset != yaw_offset_){
				yaw_offset_ = config.yaw_offset;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %.1f deg (%.3lf rad)", "yaw_offset_", yaw_offset_, yaw_offset_ * (M_PI/180.0));
			}
			if(config.test_position_offset_corrections != test_position_offset_corrections_){
				test_position_offset_corrections_ = config.test_position_offset_corrections;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %s", "test_position_offset_corrections_", test_position_offset_corrections_ ? "true" : "false");
			}
			if(config.test_velocity_offset_corrections != test_velocity_offset_corrections_){
				test_velocity_offset_corrections_ = config.test_velocity_offset_corrections;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %s", "test_velocity_offset_corrections_", test_velocity_offset_corrections_ ? "true" : "false");
			}
			if(config.debug_covariances != dbg_covariances_){
				dbg_covariances_ = config.debug_covariances;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %s", "dbg_covariances_", dbg_covariances_ ? "true" : "false");
			}
			if(config.debug_positions != dbg_positions_){
				dbg_positions_ = config.debug_positions;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %s", "dbg_positions_", dbg_positions_ ? "true" : "false");
			}
			if(config.debug_velocities != dbg_velocities_){
				dbg_velocities_ = config.debug_velocities;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %s", "dbg_velocities_", dbg_velocities_ ? "true" : "false");
			}
			if(config.debug_deltas != dbg_deltas_){
				dbg_deltas_ = config.debug_deltas;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %s", "dbg_deltas_", dbg_deltas_ ? "true" : "false");
			}
			if(config.debug_mavlink != dbg_mavlink_){
				dbg_mavlink_ = config.debug_mavlink;
				ROS_INFO_NAMED("visual_odometer", "Setting parameter \'%s\' to %s", "dbg_mavlink_", dbg_mavlink_ ? "true" : "false");
			}
		}
	}

	VisionPoseGlitchLevel get_detected_pose_glitch_level(Eigen::Vector3d current_position, Eigen::Vector3d previous_position, GlitchDetectionMetricType error_metric_type){
		VisionPoseGlitchLevel detectedGlitchLvl = NONE;
		double dx, dy, dz, dist2d, dist3d;
		dx = current_position.x() - previous_position.x();
		dy = current_position.y() - previous_position.y();
		dz = current_position.z() - previous_position.z();
		dist2d = std::sqrt(dx*dx + dy*dy);
		dist3d = std::sqrt(dx*dx + dy*dy + dz*dz);

		double poseErrMetric;
		std::string dbgMetricType = "NONE";
		if(error_metric_type == X_AXIS_ONLY){
			poseErrMetric = std::fabs(dx);
			dbgMetricType = "X_AXIS_ONLY";
		} else if(error_metric_type == Y_AXIS_ONLY){
			poseErrMetric = std::fabs(dy);
			dbgMetricType = "Y_AXIS_ONLY";
		} else if(error_metric_type == Z_AXIS_ONLY){
			poseErrMetric = std::fabs(dz);
			dbgMetricType = "Z_AXIS_ONLY";
		} else if(error_metric_type == EUCLIDEAN_DISTANCE_2D){
			poseErrMetric = dist2d;
			dbgMetricType = "2D Euclidean Distance";
		} else if(error_metric_type == EUCLIDEAN_DISTANCE_3D){
			poseErrMetric = dist3d;
			dbgMetricType = "3D Euclidean Distance";
		}

		// Determine glitch level present (if any) based on user-defined error-metric level thresholds
		if(poseErrMetric >= glitch_dist_thresh_max_err_){
			detectedGlitchLvl = SERIOUS;
			ROS_FATAL_NAMED("visual_odometer", "[Vision Pose] Major Glitch Detected --- (%s) Pose Error = %.4lf", dbgMetricType.c_str(), poseErrMetric);
			ROS_WARN_NAMED("visual_odometer", " ------ Individual Errors: dX, dY, dZ, dist2d, dist3d = %.4lf, %.4lf, %.4lf, %.4lf, %.4lf", dx, dy, dz, dist2d, dist3d);
		} else if( (poseErrMetric >= glitch_dist_thresh_mid_err_) && (poseErrMetric < glitch_dist_thresh_max_err_) ){
			detectedGlitchLvl = MODERATE;
			ROS_FATAL_NAMED("visual_odometer", "[Vision Pose] Moderate Glitch Detected --- (%s) Pose Error = %.4lf", dbgMetricType.c_str(), poseErrMetric);
			ROS_WARN_NAMED("visual_odometer", " ------ Individual Errors: dX, dY, dZ, dist2d, dist3d = %.4lf, %.4lf, %.4lf, %.4lf, %.4lf", dx, dy, dz, dist2d, dist3d);
		} else if( (poseErrMetric >= glitch_dist_thresh_min_err_) && (poseErrMetric < glitch_dist_thresh_mid_err_) ){
			detectedGlitchLvl = MINOR;
			ROS_FATAL_NAMED("visual_odometer", "[Vision Pose] Minor Glitch Detected --- (%s) Pose Error = %.4lf", dbgMetricType.c_str(), poseErrMetric);
			ROS_WARN_NAMED("visual_odometer", " ------ Individual Errors: dX, dY, dZ, dist2d, dist3d = %.4lf, %.4lf, %.4lf, %.4lf, %.4lf", dx, dy, dz, dist2d, dist3d);
		} else if(poseErrMetric < glitch_dist_thresh_min_err_){
			detectedGlitchLvl = NONE;
			ROS_DEBUG_THROTTLE_NAMED(5, "visual_odometer", "[Vision Pose] No Glitch Detected (using \'%s\' as the metric) --- Errors: dX, dY, dZ, dist2d, dist3d = %.4lf, %.4lf, %.4lf, %.4lf, %.4lf", dbgMetricType.c_str(), dx, dy, dz, dist2d, dist3d);
		} else{
			detectedGlitchLvl = NONE;
			ROS_DEBUG_THROTTLE_NAMED(5, "visual_odometer", "[Vision Pose] Undefined Glitch Detected (using \'%s\' as the metric) --- Errors: dX, dY, dZ, dist2d, dist3d = %.4lf, %.4lf, %.4lf, %.4lf, %.4lf", dbgMetricType.c_str(), dx, dy, dz, dist2d, dist3d);
		}
		return detectedGlitchLvl;
	}

	/**
	 * @brief Convert vector and covariance from local ENU to local NED frame
	 * @param stamp		ROS timestamp of the message
	 * @param vel_enu	Velocity/speed vector in the ENU frame
	 * @param cov_enu	Linear velocity/speed in the ENU frame
	 * Send transformed data from local ENU to NED frame
	 */
	void update_vision_speeds(const ros::Time &stamp, const Eigen::Vector3d &vel_enu, const ftf::Covariance3d &cov_enu){
		latest_velocities_stamp_ = stamp;
		latest_velocities_ = Eigen::Vector3d(vel_enu);
		latest_velocities_cov_ = ftf::Covariance3d(cov_enu);

		ftf::EigenMapConstCovariance6d cov_map(latest_velocities_cov_.data());
		auto urt_view = Eigen::Matrix<double, 6, 6>(cov_map.triangularView<Eigen::Upper>());

		// if(dbg_velocities_){
		// 	ROS_DEBUG_NAMED("visual_odometer",        "[Vision Velocities] Received Time = %.2lf (secs) --- Vx, Vy, Vz (m/s) == %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf",
		// 		latest_velocities_stamp_.toSec(), latest_velocities_.x(), latest_velocities_.y(), latest_velocities_.z(),
		// 		(double) urt_view(0,0), (double) urt_view(1,1), (double) urt_view(2,2)
		// 		/**(double) latest_velocities_cov_.at(0), (double) latest_velocities_cov_.at(4), (double) latest_velocities_cov_.at(9)*/
		// 	);
		// }
		if(dbg_covariances_) ROS_DEBUG_STREAM_NAMED("visual_odometer", " --------------------------- Velocity Covariance = " << urt_view);
	}

	/**
	 * @brief Send vision estimate transform to FCU position controller
	 */
	void send_vision_estimates(const ros::Time &stamp, const Eigen::Affine3d &tr, const geometry_msgs::PoseWithCovariance::_covariance_type &cov){
		// Skip computations if the current topic's data isn't new
		if(prev_pose_stamp_ == stamp){
			ROS_DEBUG_THROTTLE_NAMED(10, "visual_odometer", "Vision: Same transform as last one, dropped.");
			return;
		}

		double roll_offset = roll_offset_ * (M_PI/180.0);
		double pitch_offset = pitch_offset_ * (M_PI/180.0);
		double yaw_offset = yaw_offset_ * (M_PI/180.0);
		Eigen::Matrix3d correct_offset_rotation;
		correct_offset_rotation = Eigen::AngleAxisd(roll_offset, Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(pitch_offset, Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(yaw_offset, Eigen::Vector3d::UnitZ()
		);

		// Extract relavent data from the newly recieved visual odometer's pose
		Eigen::Vector3d position = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
		Eigen::Quaterniond quat = ftf::transform_orientation_enu_ned(
			ftf::transform_orientation_baselink_aircraft(
				Eigen::Quaterniond(tr.rotation())
			)
		);
		Eigen::Vector3d rpy = ftf::quaternion_to_rpy(quat);

		Eigen::Matrix3d cur_rotation;
		cur_rotation = Eigen::AngleAxisd(rpy.x() + roll_offset, Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(rpy.y() + pitch_offset, Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(rpy.z() + yaw_offset, Eigen::Vector3d::UnitZ()
		);
		// Eigen::Vector3d rpy = ftf::quaternion_to_rpy(ftf::transform_orientation_enu_ned(Eigen::Quaterniond(tr.rotation())));

		// Extract covariance data if available
		auto cov_ned = ftf::transform_frame_enu_ned(cov);
		ftf::EigenMapConstCovariance6d cov_map(cov_ned.data());
		auto urt_view = Eigen::Matrix<double, 6, 6>(cov_map.triangularView<Eigen::Upper>());
		// ROS_DEBUG_STREAM_NAMED("visual_odometer", "Vision: Covariance URT: " << std::endl << urt_view(0,0));

		if(test_position_offset_corrections_){ position = correct_offset_rotation.inverse() * position; }

		// If this is the first pose information recieved, update internally
		// storage of data and skip computations for this turn
		if(!last_pose_recvd){
			last_pose_recvd = true;
			prev_rpy = rpy;
			prev_position = position;
			prev_rotation = cur_rotation;
			prev_rotation_inverse = prev_rotation.inverse();
			ROS_DEBUG_THROTTLE_NAMED(10, "visual_odometer", "Vision: First VO data, skipping computations until another message received.");
			return;
		}

		/**
		 	Calculate the difference between current pose and last known pose.
		 This pose delta can be useful for detecting / mitigating undesirable
		 jumps in estimated pose due to VO losing track, or other sources for
		 error.

		 	NOTE: This used to be required for sending the VISION_POSITION_DELTA
		 msg to the pixhawk (since VISION_POSITION_ESTIMATE's weren't being used
		 used by APM's EK3)
		*/
		Eigen::Vector3d delta_pos = position - prev_position;
		Eigen::Vector3d delta_pos_corrected = prev_rotation_inverse * delta_pos;
		Eigen::Matrix3d delta_rot = prev_rotation_inverse * cur_rotation;
		Eigen::Vector3d delta_rpy = ftf::quaternion_to_rpy(Eigen::Quaterniond(delta_rot));
		// ROS_DEBUG_STREAM_NAMED("visual_odometer", "Vision: Delta Position: " << std::endl << delta_pos_corrected);
		// ROS_DEBUG_STREAM_NAMED("visual_odometer", "Vision: Delta RPY: " << std::endl << delta_rpy);

		/** Determine if there's a significant jump in the estimated pose
		 (compared to the last known pose estimate we received)
		*/
		VisionPoseGlitchLevel poseGlitchDetected;
		if(!check_pose_glitch_){ poseGlitchDetected = NONE; }
		else{ poseGlitchDetected = get_detected_pose_glitch_level(position, prev_position, glitchDetectMeth_); }

		/**
			VISION_POSITION_ESTIMATE specific calculations
		*/
		if(enable_vision_pose_estimate_mavlink_){
			mavlink::common::msg::VISION_POSITION_ESTIMATE vpp{};
			vpp.usec = (uint64_t) (stamp.toNSec() / 1000);
			vpp.x = position.x();
			vpp.y = position.y();
			vpp.z = position.z();
			vpp.roll = rpy.x() + roll_offset;
			vpp.pitch = rpy.y() + pitch_offset;
			vpp.yaw = rpy.z() + yaw_offset;
			ftf::covariance_urt_to_mavlink(cov_map, vpp.covariance);

			// If there has been enough of a jump detected in the
			// current VO-estimated pose compared to our last known pose
			// then we assume that the VO algorithm is either currently lost
			// or had previously been lost and has re-initialized at its
			// tracking origin. In this case, we send our last known pose
			// information to the autopilot's EKF as a "checkpoint" for
			// being used as a positional sensor input source.
			if(poseGlitchDetected < MODERATE){
				UAS_FCU(m_uas)->send_message_ignore_drop(vpp);
				if(dbg_mavlink_) ROS_DEBUG_NAMED("visual_odometer", "New VISION_POSITION_ESTIMATE message sent = (Time, X, Y, Z, R, P, Y) --- %d, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", (int) vpp.usec, vpp.x, vpp.y, vpp.z, vpp.roll, vpp.pitch, vpp.yaw);
				prev_vpp = vpp;
			} else{
				prev_vpp.usec = (uint64_t) (stamp.toNSec() / 1000);
				UAS_FCU(m_uas)->send_message_ignore_drop(prev_vpp);
				ROS_DEBUG_NAMED("visual_odometer", "Last known VISION_POSITION_ESTIMATE Msg sent! ");
			}
		}

		/**
			VISION_SPEED_ESTIMATE specific calculations
		*/
		if(enable_vision_velocities_mavlink_){
			Eigen::Vector3d velocities_out = latest_velocities_;
			ftf::Covariance3d velocities_out_cov = latest_velocities_cov_;

			// TODO: Prevent usage of data older last stored velocity data
			// TODO: Prevent usage of null data (determination????)
			ros::Time velocities_out_stamp = latest_velocities_stamp_;

			if(test_velocity_offset_corrections_){
				ros::Time corrected_velocities_stamp = velocities_out_stamp;
				Eigen::Vector3d corrected_velocities = cur_rotation.inverse() * latest_velocities_;
				// Eigen::Vector3d corrected_velocities = ftf::transform_frame_enu_ned(latest_velocities_);
				ftf::Covariance3d corrected_velocities_cov = ftf::transform_frame_enu_ned(velocities_out_cov);

				ftf::EigenMapConstCovariance6d cor_cov_map(corrected_velocities_cov.data());
				auto cor_urt_view = Eigen::Matrix<double, 6, 6>(cor_cov_map.triangularView<Eigen::Upper>());

				if(dbg_velocities_){
					ROS_DEBUG_NAMED("visual_odometer", "Vision Velocities being sent to APM EKF:");
					ROS_DEBUG_NAMED("visual_odometer", " -- Un-Corrected Velocities (X, Y, Z) = %.3lf, %.3lf, %.3lf (m/s)", latest_velocities_.x(), latest_velocities_.y(), latest_velocities_.z());
					ROS_DEBUG_NAMED("visual_odometer", " --    Corrected Velocities (X, Y, Z) = %.3lf, %.3lf, %.3lf (m/s)", corrected_velocities.x(), corrected_velocities.y(), corrected_velocities.z());
					if(dbg_covariances_) ROS_DEBUG_STREAM_NAMED("visual_odometer", " -- Covariance: " << cor_urt_view);
					// ROS_DEBUG_STREAM_NAMED("visual_odometer", " ------ Covariance: " << corrected_velocities_cov);
				}

				velocities_out_stamp = corrected_velocities_stamp;
				velocities_out = corrected_velocities;
				velocities_out_cov = corrected_velocities_cov;
			} else{
				ftf::EigenMapConstCovariance6d cov_map(latest_velocities_cov_.data());
				auto urt_view = Eigen::Matrix<double, 6, 6>(cov_map.triangularView<Eigen::Upper>());
				if(dbg_velocities_){
					ROS_DEBUG_NAMED("visual_odometer", "Vision Velocities (X, Y, Z) being sent to APM EKF == %.3lf, %.3lf, %.3lf (m/s)", velocities_out.x(), velocities_out.y(), velocities_out.z());
					// ROS_DEBUG_NAMED("visual_odometer", " ----- Covariance (X, Y, Z): " << velocities_out_cov);
					if(dbg_covariances_) ROS_DEBUG_STREAM_NAMED("visual_odometer", " --------------------------- Associated Covariance = " << urt_view);
					// ROS_DEBUG_STREAM_NAMED("visual_odometer", " ------ Covariance: " << velocities_out_cov);
				}
			}

			mavlink::common::msg::VISION_SPEED_ESTIMATE vs{};
			vs.usec = (uint64_t) (stamp.toNSec() / 1000);
			vs.x = velocities_out.x();
			vs.y = velocities_out.y();
			vs.z = velocities_out.z();
			ftf::covariance_to_mavlink(velocities_out_cov, vs.covariance);
			// If there has been enough of a jump detected in the
			// current VO-estimated pose compared to our last known pose
			// then we assume that the VO algorithm is either currently lost
			// or had previously been lost and has re-initialized at its
			// tracking origin. In this case, we send our last known pose
			// information to the autopilot's EKF as a "checkpoint" for
			// being used as a positional sensor input source.
			if(poseGlitchDetected < MODERATE){
				UAS_FCU(m_uas)->send_message_ignore_drop(vs);
				if(dbg_mavlink_) ROS_DEBUG_NAMED("visual_odometer", "New VISION_SPEED_ESTIMATE message sent = (Time, X, Y, Z, CovX, CovY, CovZ) --- %d, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", (int) vs.usec, vs.x, vs.y, vs.z, vs.covariance[0], vs.covariance[7], vs.covariance[14]);
				prev_vse = vs;
			} else{
				prev_vse.usec = (uint64_t) (stamp.toNSec() / 1000);
				UAS_FCU(m_uas)->send_message_ignore_drop(prev_vse);
				ROS_DEBUG_NAMED("visual_odometer", "Sending Last known VISION_SPEED_ESTIMATE Msg to EKF!");
			}
		}
		/**
			VISION_POSITION_DELTA specific calculations
		*/
		if(enable_vision_pose_delta_mavlink_){
			double confidence;
			double base_confidence = 0.0;
			double covariance2confidenceMetric;
			if(glitchDetectMeth_ == X_AXIS_ONLY){	   covariance2confidenceMetric = urt_view(0, 0); /** X-Position Covariance */ }
			else if(glitchDetectMeth_ == Y_AXIS_ONLY){ covariance2confidenceMetric = urt_view(1, 1); /** Y-Position Covariance */ }
			else if(glitchDetectMeth_ == Z_AXIS_ONLY){ covariance2confidenceMetric = urt_view(2, 2); /** Z-Position Covariance */ }
			else{ covariance2confidenceMetric = urt_view(0, 0); /** Default to using X-Position */ }

			/** Translate covariance value extracted to an equivalent pose
			 confidence value depending on certain conditions

				NOTE: The behavior of the values in the covariance matrix
			 being used can potentially be completely different and uncorrelated
			 to the norm, depending on the specific visual odometery backend
			 algorithm being used.

				TODO: Current method uses confidence-level threshold values that
			 work with ORB_SLAM2 backend, but better logic needs to be determined
			 that could better handle covariance values outside the ORB_SLAM2 range
			*/
			if( 	(covariance2confidenceMetric <= good_health_thresh_) &&
		    		(covariance2confidenceMetric != 0.0) ){ 	base_confidence = max_confidence_;
			} else if( (covariance2confidenceMetric >= bad_health_thresh_) &&
				(covariance2confidenceMetric != 0.0) ){ 	base_confidence = 0.0;
			} else if(covariance2confidenceMetric == 0.0){	base_confidence = max_confidence_;
				ROS_DEBUG_NAMED("visual_odometer", "Null Vision Pose Estimate Health (%.1lf) ---- Confidence Metric (%.5lf) == 0", confidence, covariance2confidenceMetric);
			} else{									base_confidence = default_confidence_;
				ROS_DEBUG_NAMED("visual_odometer", "Default Vision Pose Estimate Health (%.1lf) ---- Confidence Metric (%.5lf)", confidence, covariance2confidenceMetric);
			}

			// Adapt the baseline confidence depending on any pose glitches detected, or etc.
			if( (base_confidence > 0.0) && (poseGlitchDetected != NONE) ){
				if(poseGlitchDetected >= SERIOUS){
					confidence = 0.0;
				} else if( (poseGlitchDetected < SERIOUS) && (poseGlitchDetected >=  MODERATE) ){
					confidence = mid_glitch_confidence_;
				} else if( (poseGlitchDetected < MODERATE) && (poseGlitchDetected >=  MINOR) ){
					confidence = min_glitch_confidence_;
				} else if(poseGlitchDetected < MINOR){ confidence = base_confidence; }
			} else{ confidence = base_confidence; }

			// Pack and send out the extrapolated VISION_POSITION_DELTA message to the autopilot for EKF fusion
			mavlink::ardupilotmega::msg::VISION_POSITION_DELTA vp{};
			vp.time_usec = stamp.toNSec() / 1000;
			vp.time_delta_usec = vp.time_usec - (prev_pose_stamp_.toNSec() / 1000);
			vp.position_delta[0] = delta_pos_corrected.x();
			vp.position_delta[1] = delta_pos_corrected.y();
			vp.position_delta[2] = delta_pos_corrected.z();
			vp.angle_delta[0] = delta_rpy.x();
			vp.angle_delta[1] = delta_rpy.y();
			vp.angle_delta[2] = delta_rpy.z();
			vp.confidence = confidence;
			UAS_FCU(m_uas)->send_message_ignore_drop(vp);
		}
		/**
			Misc Debugging Mechanisms
		*/
		if(publish_debug_pose_estimate_){
			geometry_msgs::PoseWithCovarianceStamped pMsg;
			pMsg.header.stamp = stamp;
			pMsg.header.frame_id = tf_frame_id;
			pMsg.pose.pose.position.x = position.x();
			pMsg.pose.pose.position.y = position.y();
			pMsg.pose.pose.position.z = position.z();
			tf::quaternionEigenToMsg(quat, pMsg.pose.pose.orientation);

			pMsg.pose.covariance[0] = urt_view(0,0);
			pMsg.pose.covariance[7] = urt_view(1,1);
			pMsg.pose.covariance[14] = urt_view(2,2);
			debug_pose_pub.publish(pMsg);
		}

		// Store the newly acquired pose information internally for the next update step
		prev_rpy = rpy;
		prev_position = position;
		prev_rotation = cur_rotation;
		prev_pose_stamp_ = stamp;
		prev_rotation_inverse = prev_rotation.inverse();
	}

	/**
	* @brief Callbacks related to sending visual odometry pose positional data
	* from ROS to the autopilot's EKF to be used as an input source for fusion,
	* via Mavlink messages (e.g VISION_POSITION_ESTIMATE, VISION_POSITION_DELTA, etc.)
-	*/
	void vision_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &req){
		Eigen::Affine3d tr;
		ftf::Covariance6d cov{};	// zero initialized
		tf::poseMsgToEigen(req->pose, tr);
		publish_tf(req);
		send_vision_estimates(req->header.stamp, tr, cov);
	}
	void vision_pose_cov_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &req){
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose.pose, tr);
		send_vision_estimates(req->header.stamp, tr, req->pose.covariance);
	}

	/**
	* @brief Callbacks related to sending visual odometry pose velocity data
	* from ROS to the autopilot's EKF to be used as an input source for fusion,
	* via Mavlink messages (e.g VISION_SPEED_ESTIMATE, VISION_POSITION_DELTA??, etc.)
-	*/
	void twist_cb(const geometry_msgs::TwistStamped::ConstPtr &req){
		ftf::Covariance3d cov{};	// zero initialized
		update_vision_speeds(req->header.stamp, ftf::to_eigen(req->twist.linear), cov);
	}
	void twist_cov_cb(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &req){
		ftf::Covariance3d cov3d{};	// zero initialized
		ftf::EigenMapCovariance3d cov3d_map(cov3d.data());
		ftf::EigenMapConstCovariance6d cov6d_map(req->twist.covariance.data());
		cov3d_map = cov6d_map.block<3, 3>(0, 0); // only the linear velocity will be sent
		update_vision_speeds(req->header.stamp, ftf::to_eigen(req->twist.twist.linear), cov3d);
	}

	/**
	* @brief Callbacks related to using tf relations to recieve pose updates from ROS
-	*/
	void publish_tf(const geometry_msgs::PoseStamped::ConstPtr &msg){
		if(tf_send){
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
	void transform_cb(const geometry_msgs::TransformStamped &transform){
		Eigen::Affine3d tr;
		ftf::Covariance6d cov{};	// zero initialized
		tf::transformMsgToEigen(transform.transform, tr);
		send_vision_estimates(transform.header.stamp, tr, cov);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VisualOdometerInterfacePlugin, mavros::plugin::PluginBase)
