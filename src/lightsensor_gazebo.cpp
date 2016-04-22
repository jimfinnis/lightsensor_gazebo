/**
 * @file lightsensor_gazebo_node.cpp
 * @brief  Brief description of file.
 *
 */

#include "ros/ros.h"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"

#include "lightsensor_gazebo/LightSensor.h"

namespace gazebo {

class LightSensor : public ModelPlugin {
public:
    LightSensor(){
    }
    
    virtual ~LightSensor(){
    }
private:
    
    bool updateOK;
    // the timer sets updateOK, so that the update event is permitted
    // to do its stuff
    void timerCallback(const ros::TimerEvent&){
        updateOK=true;
    }
    
protected:    
    virtual void Load(physics::ModelPtr _model,sdf::ElementPtr _sdf){
        updateOK=false;
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                             << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package.");
            return;
        }
        
        model = _model;
        world = _model->GetWorld();
        
        // load parameters
        if (_sdf->HasElement("robotNamespace"))
            namespc = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
        else
            namespc.clear();
        
        if (_sdf->HasElement("bodyName"))
        {
            linkname = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
            link = _model->GetLink(linkname);
        }
        else
        {
            link = _model->GetLink();
            linkname = link->GetName();
        }
        
        // assert that the body by link_name_ exists
        if (!link)
        {
            ROS_FATAL("GazeboRosIMU plugin error: bodyName: %s does not exist\n", linkname.c_str());
            return;
        }
        
        if(_sdf->HasElement("interval"))
            interval = _sdf->GetElement("interval")->Get<double>();
        else 
            interval = 0.1;
        
        if(_sdf->HasElement("topic"))
            topic = _sdf->GetElement("topic")->Get<std::string>();
        else
            topic = "pixels";
        
        // get a node handle
        node = new ros::NodeHandle(namespc);
        
        // and make a timer
        timer = node->createTimer(ros::Duration(interval),
                                  &LightSensor::timerCallback,this);
        
        // and publisher(s)
        pub = node->advertise<lightsensor_gazebo::LightSensor>(topic,10);
        
        updateConnection = event::Events::ConnectWorldUpdateBegin(
                                                                  boost::bind(&LightSensor::OnUpdate,this,_1));
    }
    
    virtual void OnUpdate(const common::UpdateInfo &info){
        if(updateOK){
            lightsensor_gazebo::Pixel p;
            math::Pose myPose = link->GetWorldPose();
            
            // iterate through the objects in the world
            physics::Model_V list=world->GetModels();
            for(physics::Model_V::iterator it=list.begin();
                it!=list.end();++it){
                physics::ModelPtr ptr = *it;
                std::string name = ptr->GetName();
                printf("Name: %s, ",(*it)->GetName().c_str());
                // find those with "lightrgb" in their name
//                if(name.find("light")!=std::string::npos)
                if(name.find("unit")!=std::string::npos)
                {
                    // calculate the relative position of both corners
                    // of the AABB
                    math::Box bbox = ptr->GetBoundingBox();
                    
                    math::Vector3 v = myPose.pos - bbox.min;
//                    printf("relpos1: %f,%f ",p.pos.x,p.pos.y);
                    double angle1 = atan2(v.y,-v.x); // angle in world space
                    angle1 += myPose.rot.GetYaw(); // convert to robot space
                    angle1 = fmod(angle1+2.0*3.14145927,2*3.1415927);
                    
                    v = myPose.pos - bbox.max;
//                    printf("relpos2: %f,%f ",p.pos.x,p.pos.y);
                    double angle2 = atan2(v.y,-v.x); // angle in world space
                    angle2 += myPose.rot.GetYaw(); // convert to robot space
                    angle2 = fmod(angle2+2.0*3.14145927,2*3.1415927);
                    
                    
                    printf("angles: %f,%f\n",angle1,angle2);
                    // extract the colour - it's the three chars
                    // after "lightrgb" as hex digits
//                    int r = name.at(8);
//                    int g = name.at(9);
//                    int b = name.at(10);
                }
            }
                
            
            data.pixels.clear();
            for(int i=0;i<10;i++){
                data.pixels.push_back(p);
            }
            pub.publish(data);
            updateOK=false;
        }
    }

private:
    
    lightsensor_gazebo::LightSensor data;
    
    physics::ModelPtr model;
    physics::WorldPtr world;
    physics::LinkPtr link;
    event::ConnectionPtr updateConnection;
    std::string linkname;
    
    ros::NodeHandle *node;
    ros::Publisher pub;
    ros::Timer timer;
    std::string namespc;
    std::string topic;
    double interval;
};

GZ_REGISTER_MODEL_PLUGIN(LightSensor)
}
