#include "System.h"
#include "LogMacro.h"

#include <iostream>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <prophesee_event_msgs/Event.h>
#include <prophesee_event_msgs/EventArray.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>

using Event          = prophesee_event_msgs::Event;               //for prophess
using EventArray     = prophesee_event_msgs::EventArray;
//using Event          = dvs_msgs::Event;                              //for dvs
//using EventArray     = dvs_msgs::EventArray;

std::shared_ptr<CannyEVT::System> pSystem;
std::string bag_path;
double start_time;


void PubEventData(std::string &event_topic, std::shared_ptr<CannyEVT::System> pSystem){
  //process event data to system
  rosbag::Bag bag;

  bag.open( bag_path, rosbag::bagmode::Read );

  std::vector<std::string> topics{ event_topic };

  rosbag::View view( bag, rosbag::TopicQuery( topics ) );

  for ( rosbag::MessageInstance const m : rosbag::View( bag ) )
    {
      std::string topic = m.getTopic();

      if ( topic == event_topic )
        {
          EventArray::ConstPtr eap = m.instantiate<EventArray>();
          for ( Event e : eap->events ){
            size_t x = e.x;
            uint16_t y = e.y;
            double   ts = e.ts.toSec();
            bool     p = e.polarity;

              pSystem->GrabEventData( x, y, ts, p );
          } 
          usleep(1000);
        }
    }

  bag.close();
}



int main( int argc, char **argv )
{
  ros::init( argc, argv, "CannyEVT_node" );

  ros::NodeHandle nh;

  image_transport::ImageTransport it( nh );
  std::string config_path( argv[1] );

  INFO("[main]", "Read config file at %s", config_path.c_str());
  std::cout << std::fixed << std::setprecision(10);

  pSystem.reset( new CannyEVT::System(config_path ));

  cv::FileStorage fs( config_path, cv::FileStorage::READ );
  std::string event_topic = fs["event_topic"].string();
  std::string imu_topic   = fs["imu_topic"].string();
  bag_path = fs["bag_path"].string();
  start_time = fs["start_time"];

  //open three threads
  std::thread thd_BackEnd(&CannyEVT::System::ProcessBackEnd, pSystem);
	
  //reading data from rosbag 
	std::thread thd_PubEventData(PubEventData, std::ref(event_topic), pSystem);

  std::thread thd_Draw(&CannyEVT::System::Draw, pSystem);

  thd_BackEnd.join();
  thd_PubEventData.join();
  thd_Draw.join();

  //another way for reading data

  // ros::Subscriber sub_imu = nh.subscribe( imu_topic, 2000, GrabImu);

  // ros::Subscriber sub_event = nh.subscribe( event_topic, 500, GrabImu);


  ROS_INFO( "finished..." );

  ros::shutdown();

  return 0;
}