#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <std_msgs/Float32.h>

using namespace std;
typedef std::map<std::string, double> SkDistances;
typedef std::pair<std::string, double> SkPair;

std::vector<string> getListOfFrames(const std::string allFrames,const std::string topicName)
{
    std::vector<string> listOfFrames;
    size_t found = 0;
    do
    {
	found = allFrames.find(topicName,found+1);
	if(found!=std::string::npos)
	{
	    unsigned next_space_pos = allFrames.find(" ",found+topicName.length());
	    listOfFrames.push_back(allFrames.substr(found,next_space_pos-found));
	}
    }
    while(found!=std::string::npos);
    return listOfFrames;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "sk_closest_frame");

  ros::NodeHandle node("~");
  node.advertise<std_msgs::Float32>("min_distance",1);
  
  tf::TransformListener listener;
  
  std::string target_frame("/base_link");
  if(! node.getParam("target_frame",target_frame))
  {
    ROS_WARN_STREAM("target_frame argument not set, using default : "<<target_frame);
  }
    
  /* Skeleton definition */
  std::vector<std::string> allJoints(15);
  allJoints[0] = "/head";
  allJoints[1] = "/neck";
  allJoints[2] = "/torso";
  allJoints[3] = "/left_shoulder";
  allJoints[4] = "/left_elbow";
  allJoints[5] = "/left_hand";
  allJoints[6] = "/right_shoulder";
  allJoints[7] = "/right_elbow";
  allJoints[8] = "/right_hand";
  allJoints[9] = "/left_hip";
  allJoints[10] = "/left_knee";
  allJoints[11] = "/left_foot";
  allJoints[12] = "/right_hip";
  allJoints[13] = "/right_knee";
  allJoints[14] = "/right_foot";

  double d=-1;
  
  ros::Rate rate(1.0);

  while (node.ok()){
    ros::spinOnce();
    try{
	SkDistances sk_distances;
	const std::string allFrames = listener.allFramesAsString();
	//ROS_INFO_STREAM("A : "<<allFrames);
	for(std::vector<std::string>::iterator joint=allJoints.begin();joint!=allJoints.end();++joint)
	{
	  std::vector<std::string> joints = getListOfFrames(allFrames,*joint);
	  for(std::vector<std::string>::iterator it=joints.begin();it!=joints.end();++it)
	  {
	    std::string frame(*it);
	    tf::StampedTransform transform;
	    listener.waitForTransform(target_frame,frame,ros::Time(0),ros::Duration(1.0));
	    listener.lookupTransform(target_frame,frame,ros::Time(0),transform);
	    d = transform.getOrigin().length();
	    ROS_INFO("%s d=%f",frame.c_str(),d);
	    sk_distances.insert(SkPair(*joint,d));
	  }
	}
    }
    catch (tf::TransformException ex){
	ROS_ERROR("ERROR : %s",ex.what());
    }
    rate.sleep();
  }
  return 0;
};
