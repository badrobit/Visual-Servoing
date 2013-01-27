//---------------------------------------------------------- blob_detection.cpp
//-----------------------------------------------------------------------------
//  Description: 
//
//    This is a ROS node that is designed to perform blob detection to look for
//  objects in images that are provided through a web camera interface. It 
//  requires the use of the b-it-bots raw_usb_cam ROS node in order to properly
//  recieve the data to use.
//-----------------------------------------------------------------------------
//  Author: Matthew S Roscoe [mat.roscoe@unb.ca]
//-----------------------------------------------------------------------------
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h" 
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "geometry_msgs/Twist.h"

#include <hbrs_srvs/ReturnBool.h>

// BOOST
#include <boost/units/systems/si.hpp>

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>

// cvBlobsLib Includes.
#include <BlobResult.h>

#include <std_srvs/Empty.h>

#include <ros/package.h>
#include <ros/console.h>

// Arm Movement Stuff
#include <arm_navigation_msgs/JointLimits.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>

// The amount of time that we have to find an object in Seconds.
#define VS_TIMEOUT 30

#define VERTICLE_OFFSET 40

class raw_visual_servoing 
{

//-----------------------------------------------------------------------------
//------------------------------ PUBLIC FUNCTIONS -----------------------------
//-----------------------------------------------------------------------------
public:

  //-------------------------------------------------------- raw_visual_servoing
  //---------------------------------------------------------------------------
  //    This function is used for all of the incoming and outgoing ROS messages
  //---------------------------------------------------------------------------
  raw_visual_servoing( ros::NodeHandle &n ) : node_handler( n ), image_transporter( node_handler )
  {
    
    try 
    {
      std::string package_path = ros::package::getPath("raw_visual_servoing") + "/data/background.png";
      std::cout << "Package Path:\t" << package_path.c_str() << std::endl;  
      background_image = cvLoadImage( package_path.c_str() );
    }
    catch ( cv::Exception& e ) 
    {
          std::cout << "Could not load background image\t" << " " << e.what() << std::endl;
    }
    

    //-------------------------------------------------------------------------
    //  Get all of the joint names for the YouBot arm as well as their limits.
    //-------------------------------------------------------------------------
    XmlRpc::XmlRpcValue parameter_list;
    node_handler.getParam("/arm_controller/joints", parameter_list);
    ROS_ASSERT(parameter_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < parameter_list.size(); ++i)
    {
      ROS_ASSERT(parameter_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      arm_joint_names_.push_back(static_cast<std::string>(parameter_list[i]));
    }

    //read joint limits
    for(unsigned int i=0; i < arm_joint_names_.size(); ++i)
    {
      arm_navigation_msgs::JointLimits joint_limits;
      joint_limits.joint_name = arm_joint_names_[i];
      node_handler.getParam("/arm_controller/limits/" + arm_joint_names_[i] + "/min", joint_limits.min_position);
      node_handler.getParam("/arm_controller/limits/" + arm_joint_names_[i] + "/max", joint_limits.max_position);
      arm_joint_limits_.push_back(joint_limits);
    }
    //------------------- END OF ARM INITILIZATION ----------------------------

    // Service commands to allow this node to be started and stopped externally
    service_do_visual_serv = node_handler.advertiseService( "do_visual_servoing", &raw_visual_servoing::do_visual_servoing, this );
    ROS_INFO( "Advertised 'do_visual_servoing' service for raw_visual_servoing" );

    ROS_INFO( "Node successfully initialized" );
  }

  //------------------------------------------------------- ~raw_visual_servoing
  //---------------------------------------------------------------------------
  //   Standard Destructor.
  //--------------------------------------------------------------------------- 
  ~raw_visual_servoing()
  {
    //  OpenCV calls to destroy any HighGUI windows that may have been opened using
    //  the provided names. 
    cvDestroyWindow( "Original" );
    cvDestroyWindow( "Thresholding" ); 
    cvDestroyWindow( "Found Blobs" ); 
  }

  //------------------------------------------------------------- imageCallBack
  //---------------------------------------------------------------------------
  //    This function is the call back function for the ROS Image Message. 
  //  Each time a new image comes from the USB Camera (through raw_usb_cam)
  //  this function will be called. It is responsible for all of the blob
  //  detection as well as any processing that is applied to the images.
  //---------------------------------------------------------------------------
  void imageCallback( const sensor_msgs::ImageConstPtr& msg_ptr )
  {
    // TODO: output all the stages of visualization.

    int master_image_width = 0; 
    int master_image_height = 0; 

    double x_offset = 0; 
    double y_offset = 0; 
    double rot_offset = 0; 

    bool done_rotational_adjustment = false; 
    bool done_base_movement_adjustment = false; 
    bool done_y_base_movement_adjustment = false; 

    IplImage* cv_image  ; 
    IplImage* blob_image; 

    sensor_msgs::CvBridge opencv_bridge;
    CBlobGetOrientation get_orientation; 

    CBlob   temp_tracked_blob; 
    double  temp_tracked_blob_distance = 0; 

    double  x_threshold = 35; 
    double  y_threshold = 35;

    double maxx; 
    double minx; 
    double maxy; 
    double miny; 
    double temp_x;
    double temp_y; 
    double dist_x; 
    double dist_y; 
    double distance;

    // Covert the image from a ROS image message to a OpenCV Image (IplImage) type.
    try
    {
      cv_image = opencv_bridge.imgMsgToCv(msg_ptr, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException error)
    {
      ROS_ERROR( "Error converting from ROS image message to OpenCV IplImage" );
    }  

    IplImage* background_threshold = cvCreateImage( cvGetSize( background_image ), 8, 1 ); 
    cvCvtColor( background_image, background_threshold, CV_BGR2GRAY ); 
    cvSmooth( background_threshold, background_threshold, CV_GAUSSIAN, 7, 7 );
    cvThreshold( background_threshold, background_threshold, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU );  

    //  Obtain image properties that we require. 
    master_image_width = cv_image->width; 
    master_image_height = cv_image->height; 

    blob_image = cvCreateImage( cvGetSize( cv_image ), IPL_DEPTH_8U, cv_image->nChannels ); 

    IplImage* gray = cvCreateImage( cvGetSize( cv_image ), IPL_DEPTH_8U, 1 );
    cvCvtColor( cv_image, gray, CV_BGR2GRAY );
    cvSmooth( gray, gray, CV_GAUSSIAN, 7, 7 );
    //cvEqualizeHist( gray, gray ); 
    cvThreshold( gray, gray, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU );

    IplImage* temp_img = cvCreateImage( cvGetSize( background_image ), 8, 1); 

    //    This takes a background image (the gripper on a white background) and removes
    //  it from the current image (cv_image). The results are stored again in cv_image.
    cvSub( gray, background_threshold, gray, NULL );

    // Find any blobs that are not white. 
    CBlobResult blobs = CBlobResult( gray, NULL, 0 );

    //  Make sure they are big enough to really be considered.
    //  In this case we will use an area of AT LEAST 100 px. 
    int minimum_blob_area = ( master_image_height * master_image_width * 0.005 ); 
    int maximum_blob_area = ( master_image_height * master_image_width * 0.2 ); 
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, minimum_blob_area ); 
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, maximum_blob_area ); 
	    
    //  We will only grab the largest blob on the first pass from that point on we will look for the centroid
    //  of a blob that is closest to the centroid of the largest blob.
    if( first_pass == true )
    {
      ROS_INFO( "First Pass" ); 

      CBlob largest_blob; 
      blobs.GetNthBlob( CBlobGetPerimeter(), 0, largest_blob ); 
      maxx = largest_blob.MaxX(); 
      minx = largest_blob.MinX(); 
      maxy = largest_blob.MaxY(); 
      miny = largest_blob.MinY(); 
      tracked_x = ( ( minx + maxx ) / 2 );
      tracked_y = ( ( miny + maxy ) / 2 ); 
    
      first_pass = false; 
    }

    //  Go through all of the blobs and find the one that is the closest to the previously tracked blob.
    for( int x = 0; x < blobs.GetNumBlobs(); x++ )
    {
      CBlob  temp_blob; 

      temp_blob = blobs.GetBlob( x );

      maxx = temp_blob.MaxX(); 
      minx = temp_blob.MinX(); 
      maxy = temp_blob.MaxY(); 
      miny = temp_blob.MinY(); 
      temp_x = ( ( minx + maxx ) / 2 );
      temp_y = ( ( miny + maxy ) / 2 ); 
      dist_x = ( temp_x ) - ( tracked_x ); 
      dist_y = ( temp_y ) - ( tracked_y ); 
      distance = sqrt( ( dist_x * dist_x ) + ( dist_y * dist_y ) );

      if( temp_tracked_blob_distance == 0 )
      {
        temp_tracked_blob = temp_blob; 
        temp_tracked_blob_distance = distance; 
        tracked_x = temp_x; 
        tracked_y = temp_y;
      }
      else
      {
        if( distance < temp_tracked_blob_distance )
        {
          temp_tracked_blob = temp_blob; 
          temp_tracked_blob_distance = distance; 
          tracked_x = temp_x; 
          tracked_y = temp_y;  
        }
      }
      
      //temp_blob.FillBlob( blob_image, CV_RGB( 255, 0, 0 ) ); 
    }

    //  Draw the blob we are tracking as well as a circle to represent the centroid of that object.
    temp_tracked_blob.FillBlob( blob_image, CV_RGB( 0, 0, 255 ) );
    cvCircle( blob_image, cvPoint( tracked_x, tracked_y ), 10, CV_RGB( 255, 0, 0 ), 2 );

    double rotation = 0.0; 
    rotation = get_orientation( temp_tracked_blob );  

    x_offset = ( tracked_x ) - ( master_image_width / 2 ); 
    y_offset = ( tracked_y ) - ( (master_image_height/2) + VERTICLE_OFFSET ); 
    rot_offset = rotation; 

    //---------------------------------------------------------------------
    //-------------------- base movement control --------------------------
    //---------------------------------------------------------------------
    if( x_offset != 0 )
    {
      double move_speed = 0.0; 

      // added a buffer for a "good enough" region of interest. [14.06.2012]
      if( x_offset >= x_threshold )
      {
        // move the robot base right
        move_speed = -0.005; 
        done_base_movement_adjustment = false; 
      }
      else if( x_offset <= -x_threshold )
      {
        // move the robot left
        move_speed = 0.005; 
        done_base_movement_adjustment = false; 
      }
      else if( x_offset > -x_threshold && x_offset < x_threshold )
      {
        move_speed = 0.0;
        done_base_movement_adjustment = true;  
      }
      else
      {
        // should never happen but just in case.
        move_speed = 0.0; 
      }

      // Prepare and then send the base movement commands.
      youbot_base_velocities.linear.y = move_speed; 
      base_velocities_publisher.publish( youbot_base_velocities ); 
    }

    //---------------------------------------------------------------------
    //-------------------- base movement control --------------------------
    //---------------------------------------------------------------------
    if( y_offset != 0 )
    {
      double move_speed = 0.0; 

      if( !safe_cmd_vel_service.call( service_msg ) )
      {
        ROS_ERROR( "Visual Servoing call to is_robot_to_close_to_obstacle has failed" );
        service_msg.response.value = true; 
      }

      if( y_offset >= y_threshold )
      {
        // move the robot base right
        move_speed = -0.005; 
        done_y_base_movement_adjustment = false; 
      }
      else if( y_offset <= -y_threshold )
      {
        // move the robot left
        move_speed = 0.005; 
        done_y_base_movement_adjustment = false; 
      }
      else if( y_offset > -y_threshold && y_offset < y_threshold )
      {
        move_speed = 0.0;
        done_y_base_movement_adjustment = true;  
      }
      else if( service_msg.response.value == true )
      {
        // This will only be set when the safe_cmd_vel is telling us that it cannot 
        //  allow for movement any longer in this direction.
        move_speed = 0.0; 
        done_y_base_movement_adjustment = true; 
      }
      else
      {
        // should never happen but just in case.
        done_y_base_movement_adjustment = true; 
        move_speed = 0.0; 
      }

      // Prepare and then send the base movement commands.
      youbot_base_velocities.linear.x = move_speed; 
      base_velocities_publisher.publish( youbot_base_velocities ); 
    }
    //------------------ END OF BASE MOVEMENT CONTROL ---------------------

    //---------------------------------------------------------------------
    //--------------------- arm rotation control --------------------------
    //---------------------------------------------------------------------
    if( rot_offset != 90 || rot_offset != 270 )
    {
      double rotational_speed = 0.0; 


      if( rot_offset > 180 )
      {
          rot_offset = rot_offset - 180; 
      }

      if( ( rot_offset < 85 && rot_offset >= 0 ) || ( rot_offset < 265 && rot_offset >= 235 ) )
      {
        rotational_speed = -0.2; 
        done_rotational_adjustment = false; 
      }
      else if( rot_offset > 94 && rot_offset < 235 )
      {
        rotational_speed = 0.1; 
        done_rotational_adjustment = false; 
      }
      else
      {
        rotational_speed = 0.0; 
        done_rotational_adjustment = true; 
      }

      youbot_arm_velocities.velocities.clear();
      for(unsigned int i=0; i < arm_joint_names_.size(); ++i)
      {
        brics_actuator::JointValue joint_value;

        joint_value.timeStamp = ros::Time::now();
        joint_value.joint_uri = arm_joint_names_[i];
        joint_value.unit = to_string(boost::units::si::radian_per_second);
        
        if( i == 4 )
        {
          joint_value.value = rotational_speed;
        }
        else
        {
          joint_value.value = 0.0; 
        }

        youbot_arm_velocities.velocities.push_back(joint_value);
        arm_velocities_publisher.publish( youbot_arm_velocities );
      }
    }
    //------------------- END OF ARM ROTATION CONTROL ---------------------

    if( done_rotational_adjustment == true && done_base_movement_adjustment == true && done_y_base_movement_adjustment == true )
    {
      blob_detection_completed = true; 
      ROS_INFO( "Visual Servoing Completed." ); 
    } 

    //-------------------------------------------------------------------------
    //---------------- VISUAL OUTPUT FOR DEBUGGING ONLY -----------------------
    //-------------------------------------------------------------------------
    //    When this is being used on the robot all of the code below may be 
    //  commented out as there is no need to see what the robot is seeing. This
    //  was meant only for development purposes. Commenting it out will consume
    //  fewer resources on the robot.
    //-------------------------------------------------------------------------
    
    // Setting up fonts for overlay information.
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA);

    cvLine( blob_image,   cvPoint( 0, (master_image_height/2) + VERTICLE_OFFSET ), cvPoint( master_image_width, (master_image_height/2) + VERTICLE_OFFSET ), CV_RGB( 255, 0, 0 ), 2, 0 ); 
    cvLine( blob_image,   cvPoint( (master_image_width/2), 0 ), cvPoint( (master_image_width/2), master_image_height ), CV_RGB( 255, 0, 0 ), 2, 0 );
    cvRectangle( blob_image, cvPoint( 0, blob_image->height-40 ), cvPoint( blob_image->width, blob_image->height ), CV_RGB( 0, 0, 0 ), -1 );

    std::string x_str = "X: "; 
    x_str += boost::lexical_cast<std::string>( x_offset ); 

    std::string y_str = "Y: "; 
    y_str += boost::lexical_cast<std::string>( y_offset ); 
 
    std::string rot_str = "Rotation: "; 
    rot_str += boost::lexical_cast<std::string>( rot_offset ); 

    cvPutText( blob_image, x_str.c_str(), cvPoint( 10, blob_image->height - 10 ), &font, CV_RGB( 255, 0, 0 ) );
    cvPutText( blob_image, y_str.c_str(),  cvPoint( 185, blob_image->height - 10 ), &font, CV_RGB( 255, 0, 0 ) );
    cvPutText( blob_image, rot_str.c_str(), cvPoint( 350, blob_image->height - 10 ), &font, CV_RGB( 255, 0, 0 ) );

    cvShowImage( "Found Blobs", blob_image );   

    /**
    * DEBUGGING
    **/                          

    //cvShowImage( "Original Image", cv_image ); 
    //cvShowImage( "Gray Scale Image", gray ); 

    //-------------------------------------------------------------------------
    //----------------------- END OF VISUAL OUTPUT ----------------------------
    //-------------------------------------------------------------------------
    //cvSetZero( blob_image ); 
    

    //  Wait for user interaction.
    cvWaitKey( 10 );

    cvSetZero( gray ); 
    cvSetZero( cv_image );
    cvSetZero( blob_image );

    cvReleaseImage( &gray ); 
    cvReleaseImage( &blob_image ); 
    cvReleaseImage( &temp_img ); 
  }

  //--------------------------------------------------------------------- start
  //---------------------------------------------------------------------------
  //   Used to start up the processing of the web camera images once the node 
  //  has been told to start.
  //--------------------------------------------------------------------------- 
  bool do_visual_servoing( hbrs_srvs::ReturnBool::Request &req, hbrs_srvs::ReturnBool::Response &res )
  {
    blob_detection_completed = false; 
    first_pass = true; 

     //  Incoming message from raw_usb_cam. This must be running in order for this ROS node to run.
    image_subscriber = image_transporter.subscribe( "/usb_cam/image_raw", 1, &raw_visual_servoing::imageCallback, this );

    safe_cmd_vel_service = node_handler.serviceClient<hbrs_srvs::ReturnBool>("/is_robot_to_close_to_obstacle");

    // Velocity control for the YouBot base.
    base_velocities_publisher = node_handler.advertise<geometry_msgs::Twist>( "/cmd_vel_safe", 1 ); 

    // Velocity Control for the YouBot arm. 
    arm_velocities_publisher = node_handler.advertise<brics_actuator::JointVelocities>( "/arm_controller/velocity_command", 1 );

    ros::Time start_time = ros::Time::now(); 

    ROS_INFO("Blob Detection Enabled");

    while( ( blob_detection_completed == false ) && ros::ok() && ( (ros::Time::now() - start_time).toSec() < VS_TIMEOUT ) )
    { 
      //ROS_INFO( "Timeout: %f", ros::Time::now() - start_time  ); 
      ros::spinOnce();
    }

    if( (ros::Time::now() - start_time).toSec() < VS_TIMEOUT )
    {
      ROS_INFO( "Visual Servoing Sucessful." ); 
      res.value = true; 
    }
    else
    {
      ROS_ERROR( "Visual Servoing Failure due to Timeout" ); 
      res.value = false; 
      geometry_msgs::Twist zero_vel;
      base_velocities_publisher.publish(zero_vel);
    }

     // Turn off the image subscriber for the web camera.
    image_subscriber.shutdown(); 

    // Turn off the velocity publishers for the YouBot Arm & Base.
    arm_velocities_publisher.shutdown(); 
    base_velocities_publisher.shutdown(); 

    // Shut down any open windows.
    cvDestroyAllWindows(); 

    ROS_INFO("Blob Detection Disabled");

    return true;
  }

  //---------------------------------------------------------------------- stop
  //---------------------------------------------------------------------------
  //   Used to stop the processing of the web camera images once the node has
  //  been asked to stop. Note this does not remove the nodes service it only
  //  halts the processing.
  //--------------------------------------------------------------------------- 
  bool stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    // Turn off the image subscriber for the web camera.
    image_subscriber.shutdown(); 

    // Turn off the velocity publishers for the YouBot Arm & Base.
    arm_velocities_publisher.shutdown(); 
    base_velocities_publisher.shutdown(); 

    // Shut down any open windows.
    cvDestroyAllWindows(); 

    ROS_INFO("Blob Detection Disabled");

    return true;
  }

//-----------------------------------------------------------------------------
//--------------------- PROTECTED FUNCTIONS / VARIABLES -----------------------
//-----------------------------------------------------------------------------
protected:

  ros::NodeHandle node_handler;
  image_transport::ImageTransport image_transporter;
  image_transport::Subscriber image_subscriber;

  ros::ServiceClient  safe_cmd_vel_service;

  hbrs_srvs::ReturnBool service_msg; 

  // Topics that this node publishes to.
  ros::Publisher base_velocities_publisher;
  ros::Publisher arm_velocities_publisher;

  // base movement topic.
  geometry_msgs::Twist youbot_base_velocities;

  // Arm Joint Names.
  std::vector<std::string> arm_joint_names_;
  std::vector<arm_navigation_msgs::JointLimits> arm_joint_limits_;
  brics_actuator::JointVelocities youbot_arm_velocities;

  // Stop and start services for this ROS node.
  ros::ServiceServer service_do_visual_serv; 

  // Node status variable;
  bool blob_detection_completed; 

  //  First pass
  bool first_pass; 

  //  Tracked Centroid Values
  double tracked_x; 
  double tracked_y; 

  IplImage* background_image; 
};

//------------------------------------------------------------------------ main
//-----------------------------------------------------------------------------
//    Main Function. Should be self evident.
//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle n;
  raw_visual_servoing ic(n);
  ros::spin();
  return 0;
}
