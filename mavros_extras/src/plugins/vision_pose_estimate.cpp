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

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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
		sp_nh("~vision_pose"), roll_offset(0.0), pitch_offset(0.0), yaw_offset(0.0),
		tf_rate(10.0), px4_tf_rotate(true), send_pose_estimate(false), tf_send(false)
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		bool tf_listen;

		// tf params
		sp_nh.param("apm_use_delta", apm_use_deltas, false);
		sp_nh.param("px4_tf_rotate", px4_tf_rotate, false);
		sp_nh.param("send_pose_estimate", send_pose_estimate, false);
		sp_nh.param("good_health_thresh", good_health_thresh, 0.1);
		sp_nh.param("bad_health_thresh", bad_health_thresh, 1000.0);
		sp_nh.param("default_confidence", default_confidence, 50.0);
		sp_nh.param("offsets/roll", roll_offset, 0.0);
		sp_nh.param("offsets/pitch", pitch_offset, 0.0);
		sp_nh.param("offsets/yaw", yaw_offset, 0.0);
		sp_nh.param("tf/listen", tf_listen, false);
		sp_nh.param("tf/send", tf_send, false);
		sp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		sp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "vision_estimate");
		sp_nh.param("tf/rate_limit", tf_rate, 10.0);

		ROS_INFO_NAMED("vision_pose", "Vision Pose Offsets (Roll, Pitch, Yaw) --- %.3lf, %.3lf, %.3lf",
			roll_offset, pitch_offset, yaw_offset
		);
		if(send_pose_estimate){
			debug_pose_pub = sp_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_estimate", 10);
		}
		if (tf_listen) {
			ROS_INFO_STREAM_NAMED("vision_pose", "Listen to vision transform " << tf_frame_id
						<< " -> " << tf_child_frame_id);
			tf2_start("VisionPoseTF", &VisionPoseEstimatePlugin::transform_cb);
		}
		else {
			vision_sub = sp_nh.subscribe("pose", 10, &VisionPoseEstimatePlugin::vision_cb, this);
			vision_cov_sub = sp_nh.subscribe("pose_cov", 10, &VisionPoseEstimatePlugin::vision_cov_cb, this);
		}

		// last_pose.setZero();
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	friend class TF2ListenerMixin;
	ros::NodeHandle sp_nh;

	ros::Subscriber vision_sub;
	ros::Subscriber vision_cov_sub;
	ros::Publisher debug_pose_pub;

	bool px4_tf_rotate;
	bool apm_use_deltas;
	bool send_pose_estimate;
	double good_health_thresh;
	double bad_health_thresh;
	double default_confidence;
	std::string tf_frame_id;
	std::string tf_child_frame_id;
	double tf_rate;
	ros::Time last_transform_stamp;
	Eigen::Affine3d last_pose;
	Eigen::Vector3d prev_position;
	Eigen::Matrix3d prev_rotation;
	Eigen::Matrix3d prev_rotation_inverse;
	bool last_pose_recvd = false;
	double roll_offset;
	double pitch_offset;
	double yaw_offset;
	bool tf_send;

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

		if(!apm_use_deltas){
			Eigen::Vector3d position;
			Eigen::Vector3d rpy;
			// Eigen::Quaterniond quats = Eigen::Quaterniond(tr.rotation());
			if(px4_tf_rotate){
				position = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
				rpy = ftf::quaternion_to_rpy(ftf::transform_orientation_enu_ned(ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation()))));
			} else{
				position = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
				rpy = ftf::quaternion_to_rpy(ftf::transform_orientation_enu_ned(Eigen::Quaterniond(tr.rotation())));
			}
			ROS_DEBUG_NAMED("vision_pose", "Vision Pose Un-Corrected R, P, Y --- %.5lf, %.5lf, %.5lf",
				rpy.x(), rpy.y(), rpy.z()
			);
			double corrected_roll = rpy.x() - roll_offset;
			double corrected_pitch = rpy.y() - pitch_offset;
			double corrected_yaw = rpy.z() - yaw_offset;
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
			cur_rotation = Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
						* Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY())
						* Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ());
			if(!last_pose_recvd){
				prev_position = position;
				prev_rotation = cur_rotation;
				prev_rotation_inverse = prev_rotation.inverse();
				last_pose_recvd = true;
				ROS_DEBUG_THROTTLE_NAMED(10, "vision_pose", "Vision: Same Pose as last one, dropped.");
				return;
			}

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
			if((cov_health_metric <= good_health_thresh) && (cov_health_metric != 0.0) ){
				confidence = 100.0;
				ROS_DEBUG_NAMED("vision_pose", "Highest Vision Pose Estimate Health (%.1lf) ---- Confidence Metric (%.5lf) <= Good Health Threshold (%.5lf)", confidence, cov_health_metric, good_health_thresh);
			} else if((cov_health_metric >= bad_health_thresh) && (cov_health_metric != 0.0) ){
				confidence = 0.0;
				ROS_DEBUG_NAMED("vision_pose", "Lowest Vision Pose Estimate Health (%.1lf) ---- Confidence Metric (%.5lf) >= Bad Health Threshold (%.5lf)", confidence, cov_health_metric, bad_health_thresh);
			} else if(cov_health_metric == 0.0){
				confidence = 100.0;
				ROS_DEBUG_NAMED("vision_pose", "Null Vision Pose Estimate Health (%.1lf) ---- Confidence Metric (%.5lf) == 0", confidence, cov_health_metric);
			} else{
				confidence = default_confidence;
				ROS_DEBUG_NAMED("vision_pose", "Default Vision Pose Estimate Health (%.1lf) ---- Confidence Metric (%.5lf)", confidence, cov_health_metric);
			}


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
			UAS_FCU(m_uas)->send_message_ignore_drop(vp);
			// ROS_DEBUG_STREAM_NAMED("vision_pose", "VISION_POSITION_DELTA Msg sent! " << std::endl);
			if(send_pose_estimate){
				// geometry_msgs::Quaternion quat;
				Eigen::Quaterniond quat {};
				geometry_msgs::PoseWithCovarianceStamped pMsg;

				ROS_DEBUG_STREAM_NAMED("vision_pose", "Vision: Covariance URT: " << std::endl << urt_view);

				// header
				pMsg.header.stamp = stamp;
				pMsg.header.frame_id = tf_frame_id;
				// pose
				pMsg.pose.pose.position.x = position.x();
				pMsg.pose.pose.position.y = position.y();
				pMsg.pose.pose.position.z = position.z();
				// quat = Eigen::Quaterniond(cur_rotation);
				quat = Eigen::Quaterniond(
					ftf::transform_orientation_enu_ned(
						ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())
				)));
				tf::quaternionEigenToMsg(quat, pMsg.pose.pose.orientation);
				// quat = tf2::toMsg(ftf::quaternion_from_rpy(rpy.x(), rpy.y(), rpy.z()));
				pMsg.pose.covariance[0] = urt_view(0,0);	// x
				pMsg.pose.covariance[7] = urt_view(1,1);	// y
				pMsg.pose.covariance[14] = urt_view(2,2);

				// publish
				debug_pose_pub.publish(pMsg);

				// mavlink::common::msg::VISION_POSITION_ESTIMATE vpp{};
				// vpp.usec = (uint64_t) (stamp.toNSec() / 1000);
				// vpp.x = position.x();
				// vpp.y = position.y();
				// vpp.z = position.z();
				// vpp.roll = rpy.x();
				// vpp.pitch = rpy.y();
				// vpp.yaw = rpy.z();
				// ftf::covariance_urt_to_mavlink(cov_map, vpp.covariance);
				// UAS_FCU(m_uas)->send_message_ignore_drop(vpp);
			}

			// last_pose = tr;
			last_transform_stamp = stamp;
		}

		// [[[end]]] (checksum: 2048daf411780847e77f08fe5a0b9dd3)

	}

	/* -*- callbacks -*- */

	/* common TF listener moved to mixin */

	void transform_cb(const geometry_msgs::TransformStamped &transform)
	{
		Eigen::Affine3d tr;
		tf::transformMsgToEigen(transform.transform, tr);
		ftf::Covariance6d cov {};	// zero initialized

		send_vision_estimate(transform.header.stamp, tr, cov);
	}

	void vision_cb(const geometry_msgs::PoseStamped::ConstPtr &req)
	{
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose, tr);
		ftf::Covariance6d cov {};	// zero initialized
		publish_tf(req);
		send_vision_estimate(req->header.stamp, tr, cov);
	}

	void vision_cov_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &req)
	{
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose.pose, tr);

		send_vision_estimate(req->header.stamp, tr, req->pose.covariance);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VisionPoseEstimatePlugin, mavros::plugin::PluginBase)
