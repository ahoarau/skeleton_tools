#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>

using namespace std;
typedef std::vector<std::string> VString;
typedef std::map<std::string, tf::StampedTransform> SkTf;
typedef std::pair<std::string, tf::StampedTransform> SkPair;

struct CompareSecond
{
    bool operator()(const SkPair& left, const SkPair& right) const
    {
        return left.second.getOrigin().length() < right.second.getOrigin().length();
    }
};


SkPair getMin(const SkTf& mymap)
{
    return *min_element(mymap.begin(), mymap.end(), CompareSecond());
}


void getListOfFrames(const std::string allFrames,const std::string topicName, VString& listOut)
{
    size_t found = 0;
    do
    {
        found = allFrames.find(topicName,found+1);
        if(found!=std::string::npos)
        {
            unsigned next_space_pos = allFrames.find(" ",found+topicName.length());
            listOut.push_back(allFrames.substr(found,next_space_pos-found));
        }
    }
    while(found!=std::string::npos);
}

void getSKFramesOnly(const VString allFrames,const std::string desFrame, VString& listOut)
{
    for(VString::const_iterator it=allFrames.begin();it!=allFrames.end();++it)
        if (it->find(desFrame) != std::string::npos)
            listOut.push_back(*it);
}

void getSKVectors(const std::string& target_frame,const std::string& base_frame,const tf::TransformListener& listener, SkTf& skTfOut)
{
    VString allFrames,SKFramesOnly;
    listener.getFrameStrings(allFrames);
    getSKFramesOnly(allFrames,base_frame,SKFramesOnly);

    for(VString::iterator it=SKFramesOnly.begin();it!=SKFramesOnly.end();++it)
    {
        const std::string frame(*it);
        tf::StampedTransform transform;
        listener.lookupTransform(target_frame,frame,ros::Time(0),transform);
        skTfOut.insert(SkPair(frame,transform));
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "sk_closest_frame");

    ros::NodeHandle node("~");
    ros::Publisher vector_pub = node.advertise<geometry_msgs::Vector3>("vector_closest_frame",1);
    ros::Publisher string_pub = node.advertise<std_msgs::String>("closest_frame",1);

    tf::TransformListener listener;

    std::string target_frame("/base_link");
    if(! node.getParam("target_frame",target_frame))
        ROS_WARN_STREAM("Target frame argument not set, using default : "<<target_frame);
    else
        ROS_INFO_STREAM("Target frame is "<<target_frame);
    /* Skeleton definition */
    const std::string J[15] =
    {
        "/head",
        "/neck",
        "/torso",
        "/left_shoulder",
        "/left_elbow",
        "/left_hand",
        "/right_shoulder",
        "/right_elbow",
        "/right_hand",
        "/left_hip",
        "/left_knee",
        "/left_foot",
        "/right_hip",
        "/right_knee",
        "/right_foot"
    };
    const VString allJoints(J,J+15);

    ros::Rate rate(100.0);

    while (node.ok()){
        try{
            SkTf skTf;
            for(VString::const_iterator it=allJoints.begin();it!=allJoints.end();++it)
            {
                getSKVectors(target_frame,*it,listener,skTf);
            }
            if(skTf.size() > 0)
            {
                ROS_INFO_ONCE("Receiving skeleton frame from the tracker.");
                SkPair min = getMin(skTf);
                geometry_msgs::Vector3 Vout;
                Vout.x = min.second.getOrigin().x();
                Vout.y = min.second.getOrigin().y();
                Vout.z = min.second.getOrigin().z();
                vector_pub.publish(Vout);

                std_msgs::String Sout;
                Sout.data = min.first;
                string_pub.publish(Sout);
                ROS_INFO_ONCE("Publishing distance vector topic ~vector_closest_frame (in %s frame) and frame name ~closest_frame ",target_frame.c_str());
            }else{
                ROS_WARN_THROTTLE(1,"No skeleton data received from the tracker. Did you launch the tracker ?");
            }
        }
        catch (tf::TransformException ex){
            ROS_ERROR_THROTTLE(1,"%s",ex.what());
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
