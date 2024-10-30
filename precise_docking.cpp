#include <map>
#include <vector>
#include "tf/tf.h"
#include <iostream>
#include "precisedocking/coorconv.h"
#include "ros/ros.h"
#include <algorithm>
#include "ros/time.h"
#include "Eigen/Eigen"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <boost/thread/thread.hpp>
#include "precisedocking/pubmsg.h"
#include "precisedocking/conversion.h"
#include "precisedocking/chccgi610nav.h"
#include "precisedocking/AprilTagDetection.h"
#include "precisedocking/AprilTagDetectionArray.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <queue>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <array>

#include <message_filters/sync_policies/approximate_time.h>

#define FUDU 57.29577951
#define loading_in_tag 1     //进入装载区的标签
#define loading_out_tag 10   //离开装载区的标签
#define weighting_in_tag 11  //进入过磅区的标签
#define weighting_out_tag 18 //离开过磅区的标签
std::string final_msg_out_topic_;  //tag数据输出话题
precisedocking::pubmsg pmsg; 
typedef message_filters::sync_policies::ApproximateTime<precisedocking::pubmsg, precisedocking::pubmsg> syncPolicy;


class SubscribeAndPublish
{
public:
    SubscribeAndPublish(ros::NodeHandle &_nh);
    //void callbackA(const precisedocking::chccgi610nav::ConstPtr &chccgi);
    //void callbackB(const precisedocking::pubmsg::ConstPtr &odomsg);
    //void callbackC(const precisedocking::pubmsg::ConstPtr &tagmsg);
    void callbackBC(const precisedocking::pubmsgConstPtr &odomsg, const precisedocking::pubmsg::ConstPtr &tagmsg);
    //void callbackAB(const precisedocking::chccgi610nav::ConstPtr &chccgi , const nav_msgs::OdometryConstPtr &odomsg);
private:
    ros::NodeHandle n_;
    ros::Publisher pub_merge;
    ros::Publisher pub_nav;
    //ros::Subscriber sub_A;
    //ros::Subscriber sub_B;
    //ros::Subscriber sub_C;
   // ros::Subscriber sub_BC;
};

SubscribeAndPublish::SubscribeAndPublish(ros::NodeHandle &_nh)
{
    printf("SubscribeAndPublish\n");
    n_ = _nh;
    //发布话题
    pub_merge = n_.advertise<precisedocking::pubmsg>(final_msg_out_topic_, 1000);
    //nodeA
    //sub_A = n_.subscribe<precisedocking::chccgi610nav>("/chccgi610_nav",100,&SubscribeAndPublish::callbackA,this);
    //sub_B = n_.subscribe< precisedocking::pubmsg>("/lio_pose",100,&SubscribeAndPublish::callbackB,this);
    //sub_C = n_.subscribe<precisedocking::pubmsg>("/apr_pose",100,&SubscribeAndPublish::callbackC,this);
    message_filters::Subscriber<precisedocking::pubmsg> sub_B(n_, "/lio_pose", 1000);
    message_filters::Subscriber<precisedocking::pubmsg> sub_C(n_, "/apr_pose", 1000);

    
   //
  
    //typedef message_filters::TimeSynchronizer<precisedocking::pubmsg, precisedocking::pubmsg> TimeSync; 
//     boost::shared_ptr<TimeSync> syncBC(new TimeSync(sub_B, sub_C,10));
//    //typedef message_filters::Synchronizer<syncPolicy> syncBC(syncPolicy(10), sub_B, sub_C);
//     syncBC->registerCallback(boost::bind(&SubscribeAndPublish::callbackBC, this, _1, _2));
    printf("Subscriber\n");

    message_filters::Synchronizer<syncPolicy> syncBC(syncPolicy(10), sub_B, sub_C);
    syncBC.registerCallback(boost::bind(&SubscribeAndPublish::callbackBC, this, _1, _2));
    printf("registerCallback\n");
    ros::MultiThreadedSpinner s(4); //多线程
    ros::spin(s);
}

void SubscribeAndPublish::callbackBC(const precisedocking::pubmsgConstPtr &liomsg,const precisedocking::pubmsg::ConstPtr &aprmsg)
{
    printf("1111\n");
    pmsg.header=liomsg->header;
    pmsg.gpstime=liomsg->gpstime;
    pmsg.gpsweek=liomsg->gpsweek;
    pmsg.heading=liomsg->heading;
    pmsg.pitch=liomsg->pitch;
    pmsg.roll=liomsg->roll;
    pmsg.alt=liomsg->alt;
    pmsg.status=liomsg->status;

    double lio_lon = liomsg->lon;
    double lio_lat = liomsg->lat;
    double apr_lon = aprmsg->lon;
    double apr_lat = aprmsg->lat;
    //int tag_dist = tagmsg->dist;

    std::cout << "lio_lon:" <<lio_lon<< std::endl;
    std::cout << "lio_lat:" <<lio_lat<< std::endl;
    std::cout << "apr_lon:" <<apr_lon<< std::endl;
    std::cout << "apr_lat:" <<apr_lat<< std::endl;
    //double pub_lon=(lio_lon+apr_lon)/2;
   // double pub_lat=(lio_lat+apr_lat)/2; 

    pmsg.lon=(lio_lon * aprmsg->error/(liomsg->error+aprmsg->error)+ apr_lon * liomsg->error/(liomsg->error+aprmsg->error));
    pmsg.lat=(lio_lat * aprmsg->error/(liomsg->error+aprmsg->error)+ apr_lat * liomsg->error/(liomsg->error+aprmsg->error));
    //pmsg.lon=(lio_lon+apr_lon)/2;
    //pmsg.lat=(lio_lat+apr_lat)/2;  
    pub_merge.publish(pmsg);


}




int main(int argc, char *argv[])
{
//gps

//float lio_z=0;

    ros::init(argc, argv, "precisedocking_process");
    ros::NodeHandle n;
    ros::param::get("~final_msg_out_topic", final_msg_out_topic_);
    ROS_INFO("Ready!");

    SubscribeAndPublish test(n);
    return 0;
}
