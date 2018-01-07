#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <string>

// Chaînes constantes
const std::string ELIKOS_ARENA_ORIGIN = "elikos_corner";
const std::string ELIKOS_LOCAL_ORIGIN = "elikos_local_origin";
const std::string ELIKOS_FCU = "elikos_fcu";
const std::string ELIKOS_VISION = "elikos_vision";
const std::string ELIKOS_ATTITUDE = "elikos_attitude";
const std::string TOPIC_VN_100 = "/vn100/imu/imu";

// Variables
tf::Quaternion attitude_;
tf::Quaternion attitude_offset_;
bool isInit_ = false;
bool lookupDone = false;
bool reInit_ = false;
tf::Vector3 pos_offset_;

// Service d'initialisation pour elikos_origin_init.
bool initialize(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
  if (isInit_) 
  {
    reInit_ = true;
  }
  isInit_ = true;
  return true;
}

// Callback pour l'IMU vn100.
void imu_callback(const sensor_msgs::Imu::ConstPtr& input)
{
	tf::quaternionMsgToTF(input->orientation, attitude_);
}


int main(int argc, char* argv[])
{
  // Initialisation
  ros::init( argc, argv, "elikos_origin_init" );
  ros::NodeHandle n;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::StampedTransform arenaOriginTransform;
  ros::Subscriber sub_;

  // Enregistrement du callback de l'IMU vn100
  sub_ = n.subscribe(TOPIC_VN_100, 1, imu_callback);

  // Envoie d'une vision initiale pour obtenir un premier elikos_fcu.
  while (!tf_listener_.canTransform(ELIKOS_LOCAL_ORIGIN, ELIKOS_FCU, ros::Time(0))) 
  {
      // Attention: Le 0.14 est l'altitude de départ du fcu.
      tf_broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(0.0, 0.0, 0.14)), 
          ros::Time::now(), ELIKOS_LOCAL_ORIGIN, ELIKOS_VISION));
  }

  // Obtention du fcu initial.
  tf::StampedTransform initialFcu;
  try {
    tf_listener_.waitForTransform(ELIKOS_LOCAL_ORIGIN, ELIKOS_FCU, ros::Time::now(), ros::Duration(5.0));
    tf_listener_.lookupTransform(ELIKOS_LOCAL_ORIGIN, ELIKOS_FCU, ros::Time(0), initialFcu);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("Origin init failed!!!! Exception : %s",ex.what());
  }

  // Le noeud est prêt à se faire initialiser.
  ros::ServiceServer service = n.advertiseService("elikos_origin_init", initialize);

  ros::Rate r(10);
  while(ros::ok())
  {
    // Ce premier bloc permet d'initialiser la position à plusieurs reprises.
    if(reInit_)
    {
      tf::StampedTransform temp;
      tf_listener_.lookupTransform(ELIKOS_ARENA_ORIGIN, ELIKOS_FCU, ros::Time(0), temp);
      pos_offset_.setX(pos_offset_.getX() - temp.getOrigin().getX());
      pos_offset_.setY(pos_offset_.getY() - temp.getOrigin().getY());
      reInit_ = false;
    }
    // C'est tellement laid.
    // Ce bloc est exécuter lorsque l'initalisation est complétée.
    if(isInit_)
    {
      // On fait un lookup une fois de la transform entre elikos_local_origin et elikos_fcu. 
      // On suppose ici que le quad a été placé à sa position de départ, il s'agit donc de la transformée entre elikos_local_origin
      // et le point de départ.
      if(!lookupDone)
      {
        try {
          // Obtention de la transform
          tf_listener_.waitForTransform(ELIKOS_LOCAL_ORIGIN, ELIKOS_FCU, ros::Time(0), ros::Duration(1.0));
    			tf_listener_.lookupTransform(ELIKOS_LOCAL_ORIGIN, ELIKOS_FCU, ros::Time(0), arenaOriginTransform);
          tf::Vector3 origin = initialFcu.getOrigin();
          // Remise de Z à 0: les deux référentiels sont au sol.
          origin.setZ(0);
          // Extraction du yaw seulement: les deux référentiels sont sur le même plan.
          tf::Quaternion rotation = initialFcu.getRotation();
          double yaw = tf::getYaw(rotation);
          rotation.setRPY(0,0,yaw);
          ROS_INFO_STREAM("Initialisation : Yaw diff is : "<<yaw);
          arenaOriginTransform.setOrigin(origin);
          arenaOriginTransform.setRotation(rotation);
          lookupDone = true;
          // Initialisation de l'offset d'attitude entre le vn100 et le référenciel de départ.
          attitude_offset_ = attitude_;
    		}
    		catch (tf::TransformException &ex) {
    			ROS_ERROR("Origin init failed!!!! Exception : %s",ex.what());
          tf_broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(0.0, 0.0, 0.17)), 
              ros::Time::now(), ELIKOS_ARENA_ORIGIN, ELIKOS_VISION));
    		}
      }
      // Publication en continue de la transform entre de la position de départ.
      tf::Transform final_arena_transform = arenaOriginTransform.inverse();
      final_arena_transform.setOrigin(arenaOriginTransform.inverse().getOrigin() + pos_offset_);
      tf_broadcaster_.sendTransform(tf::StampedTransform(final_arena_transform, ros::Time::now(), ELIKOS_ARENA_ORIGIN, ELIKOS_LOCAL_ORIGIN));
      // Publication de l'attitude du VN100 corrigée pour être dans le bon référentiel.
      tf_broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(attitude_offset_.inverse() * attitude_, tf::Vector3(0,0,0)), ros::Time::now(), ELIKOS_ARENA_ORIGIN, ELIKOS_ATTITUDE));
    }
    else
    {
      // En attendant l'appel du service, le point de départ est superposé au local_origin.
      tf_broadcaster_.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now(), ELIKOS_ARENA_ORIGIN, ELIKOS_LOCAL_ORIGIN));
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
