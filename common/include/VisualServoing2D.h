#ifndef VISUALSERVOING2D_H_
#define VISUALSERVOING2D_H_

// ROS Includes
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <hbrs_srvs/ReturnBool.h>
#include <arm_navigation_msgs/JointLimits.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>

// OpenCV Includes
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

// cvBlobsLib Includes.
#include <BlobResult.h>

// BOOST
#include <boost/units/systems/si.hpp>
#include <string>

/**
 *	This is the class that is responsible for performing visual servoing on
 *	2 Dimensional images typically provided in the RGB spectrum. We are
 *	able to also take in and deal with Black & White images.
 *
 * We take the largest blob that we can find in the image and then we try to
 * clean it up and then to track it through the environment. While tracking it
 * as the robot moves through the environment we compute required velocities
 * which will be used in order to account for the offsets in various coordinates.
 */
class VisualServoing2D
{
public:
	/**
	 * This is the constructor for the 2D Visual Servoing system that sets up all of the
	 * variables that the visual servoing system requires in order to function correctly.
	 *
	 * Modes:
	 * 0 - Standard Visual Servoing
	 * 1 - Conveyer Belt Visual Servoing
	 */
	VisualServoing2D( bool debugging,
					  int mode,
					  ros::ServiceClient safe_cmd_vel_service,
					  std::vector<std::string> arm_joint_names );
	/**
	 * Standard C++ destructor method.
	 */
	virtual ~VisualServoing2D();

	/**
	 * This function takes in a provided image and performs visual servoing on
	 * the provided image. This
	 */
	bool VisualServoing( IplImage* input_image );

private:

	/**
	 * This function takes in a given x offset in a standard Cartesian coordinate
	 * system. It will determine the direction to move the robot base to account
	 * for the provided offset.
	 */
	bool BaseAdjustmentX( double x_offset );

	/**
	 * This function takes in a given y offset in a standard Cartesian coordinate
	 * system. It will determine the direction to move the robot base to account
	 * for the provided offset.
	 */
	bool BaseAdjustmentY( double y_offset );

	/**
	 * This function is designed to take the determined rotational offset that
	 * has been previously determined and will use it to determine how the arm
	 * should be moved in order to account for the difference.
	 */
	bool ArmAdjustment( double orientation );

	/**
	 * This function loads in the background image that will be subtracted from the incoming image
	 * during the visual servoing to allow the system to better focus on non-standard parts of the
	 * image.
	 */
	IplImage* LoadBackgroundImage();

	/**
	 * This function creates the publishers that will publish velcities for both the robotic base
	 * through the GeometryTwist message as well as for the arm based on the arm model.
	 *
	 * Arm Models:
	 * 0 - Unknown.
	 * 1 - KUKA YouBot Arm
	 * 2 - KUKA Lightweight Arm.
	 */
	void CreatePublishers( int arm_model );

	/**
	 * This function takes in an image and it crops it so that it is done according to the provided
	 * scaling factor from (0.0 - 1.0).
	 */
	IplImage* RegionOfInterest( IplImage* input_image, double scale );

	/**
	 * This is a function that will take in an arbitrary number of images and create a display for
	 * them that will serve as the Heads Up Display (HUD) of the Visual Servoing Application.
	 * This is a modified version of the source code found here:
	 * http://opencv.willowgarage.com/wiki/DisplayManyImages
	 */
	void HUD(char* title, int nArgs, ...);

protected:
	/*
	 * Global Variable.
	 */
	bool 											g_debugging;
	int												g_operating_mode;

	/*
	 * Modifiable Class Level Variables.
	 */
	bool 											m_first_pass;
	bool 											m_done_base_x_adjustment;
	bool 											m_done_base_y_adjustment;
	bool 											m_done_arm_rot_adjustment;
	bool 											m_blob_detection_completed;

	int												m_image_height;
	int												m_image_width;

	double 											m_tracked_x;
	double 											m_tracked_y;

	geometry_msgs::Twist 							m_youbot_base_velocities;
	brics_actuator::JointVelocities 				m_youbot_arm_velocities;
	std::vector<std::string> 						m_arm_joint_names;

	ros::Publisher 									m_base_velocities_publisher;
	ros::Publisher 									m_arm_velocities_publisher;
	ros::NodeHandle 								m_node_handler;

	ros::ServiceClient  							m_safe_cmd_vel_service;
	hbrs_srvs::ReturnBool							m_service_msg;

	IplImage* 										m_background_image;

	/*
	 * Constant values.
	 */
	const static int								m_min_blob_area = 2500;
	const static int								m_max_blob_area = 38500;
	const static int 								m_verticle_offset = 30;
	const static double 							m_x_velocity = 0.007;
	const static double 							m_y_velocity = 0.007;
	const static double 							m_rot_velocity = 0.2;

	const static int								m_x_target = 0;
	const static int 								m_x_threshold = 60;

	const static int								m_y_target = 0;
	const static int 								m_y_threshold = 40;

	const static int								m_rot_target = 90;
	const static int								m_rot_tolerance = 5;
};

#endif /* VISUALSERVOING2D_H_ */
