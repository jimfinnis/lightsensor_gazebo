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
const double PI = 3.1415927;


// distance->brightness from experimental data
/*
              Estimate Std. Error t value Pr(>|t|)    
(Intercept)  8.201e-02  5.817e-04  140.98   <2e-16 ***
d$dist      -9.250e-02  2.832e-03  -32.66   <2e-16 ***
I(d$dist^2)  8.385e-02  4.199e-03   19.97   <2e-16 ***
I(d$dist^3) -3.995e-02  2.637e-03  -15.15   <2e-16 ***
I(d$dist^4)  9.108e-03  7.371e-04   12.36   <2e-16 ***
I(d$dist^5) -7.897e-04  7.564e-05  -10.44   <2e-16 ***
 */
double getBright(double x){
    // get the predicted "powerin"
    double y =
8.201e-2 +
-9.250e-2 * x+
8.385e-2 * x*x+
-3.995e-2 * x*x*x+
9.108e-3 * x*x*x*x+
-7.897e-4 * x*x*x*x*x;
    y/=0.005; // get actual total light in
    printf("Bright: %f\n",y);
    return y;
}


// blur code copied from bridgeserver and modified
// for doubles. Should have templated it.

#define KSIZE 5
#define HALFK ((KSIZE-1)/2)
float k[] = {0.402620,0.244201,0.054489}; // k=5, sigma=1


void blurnorm(double *out,double *p,double norm,int ch,int n){
    double sum=0;
    for(int i=0;i<n;i++){
        double t = 0;
        for(int j=-HALFK;j<=HALFK;j++){
            int px = (i+j+n)%n;
            t+= p[px*3+ch]*k[j<0?-j:j];
        }
        t /= (double)KSIZE;
        out[i*3+ch]=t;
        sum+=t;
    }
    
    // now make the values sum to norm
    double normfac = norm/sum;
    for(int i=0;i<n;i++){
        out[i*3+ch]*=normfac;
    }
}

namespace gazebo {

class LightSensor : public ModelPlugin {
public:
    LightSensor(){
        pixelStore = NULL;
    }
    
    virtual ~LightSensor(){
        if(pixelStore){
            delete[] pixelStore;
            delete[] blurbuf;
        }
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
            ROS_FATAL("lightsensor_gazebo plugin error: bodyName: %s does not exist\n", linkname.c_str());
            return;
        }
        
        if(_sdf->HasElement("interval"))
            interval = _sdf->GetElement("interval")->Get<double>();
        else 
            interval = 0.1;
        
        if(_sdf->HasElement("topic"))
            topic = _sdf->GetElement("topic")->Get<std::string>();
        else
            topic = "light";
        
        if(_sdf->HasElement("pixels"))
            pixelCt = _sdf->GetElement("pixels")->Get<int>();
        else
            pixelCt = 100;
        
        // get a node handle
        node = new ros::NodeHandle(namespc);
        
        // and make a timer
        timer = node->createTimer(ros::Duration(interval),
                                  &LightSensor::timerCallback,this);
        
        // and publisher(s)
        pub = node->advertise<lightsensor_gazebo::LightSensor>(topic,10);
        
        updateConnection = event::Events::ConnectWorldUpdateBegin(
                                                                  boost::bind(&LightSensor::OnUpdate,this,_1));
        
        pixelStore = new double[pixelCt*3];
        blurbuf = new double[pixelCt*3];
    }
    
    inline int hex2int(char c){
        return (c>'a') ? (c-'a')+10 : c-'0';
    }
    
    virtual void OnUpdate(const common::UpdateInfo &info){
        if(updateOK){
            lightsensor_gazebo::Pixel p;
            math::Pose myPose = link->GetWorldPose();
            double distance;
            
            // zero the temporary pixel store
            for(int i=0;i<pixelCt*3;i++)
                blurbuf[i]=0;
            
            // iterate through the objects in the world
            physics::Model_V list=world->GetModels();
            for(physics::Model_V::iterator it=list.begin();
                it!=list.end();++it){
                physics::ModelPtr ptr = *it;
                std::string name = ptr->GetName();
//                printf("Name: %s, ",(*it)->GetName().c_str());
                
                // find those with "lightrgb" in their name
                // DISTANCE CALC ASSUMES THERE'S ONLY ONE
                if(name.find("lightrgb")!=std::string::npos)
                {
                    // work out the distance of the emitter
                    // (not necessarily a box). We go from the
                    // centre.
                    math::Pose boxpose = ptr->GetWorldPose();
                    double boxx = boxpose.pos.x - myPose.pos.x;
                    double boxy = boxpose.pos.y - myPose.pos.y;
                    distance = sqrt(boxx*boxx+boxy*boxy);
                    
                    
                    printf("Distance: %f\n",distance);
                    // calculate the relative position of both corners
                    // of the AABB
                    math::Box bbox = ptr->GetBoundingBox();
                    
                    math::Vector3 v = myPose.pos - bbox.min;
                    printf("relpos1: %f,%f ",v.x,v.y);
                    double angle1 = atan2(v.y,-v.x); // angle in world space
                    angle1 += myPose.rot.GetYaw(); // convert to robot space
                    
                    v = myPose.pos - bbox.max;
                    printf("relpos2: %f,%f ",v.x,v.y);
                    double angle2 = atan2(v.y,-v.x); // angle in world space
                    
                    
                    angle2 += myPose.rot.GetYaw(); // convert to robot space
                    angle1 = fmod(angle1+6.0*PI,2.0*PI);
                    angle2 = fmod(angle2+6.0*PI,2.0*PI);
                    printf("angles: %f,%f\n",angle1,angle2);
                    
                    // rescale the angles to 0-1
                    angle1 *= 1.0/(2.0*PI);
                    angle2 *= 1.0/(2.0*PI);
                    
                    // extract the colour - it's the three chars
                    // after "lightrgb" as hex digits
                    double r = hex2int(name.at(8))*16;
                    double g = hex2int(name.at(9))*16;
                    double b = hex2int(name.at(10))*16;
                    
                    // work out the start and end pixels
                    int p1 =(int)(angle1*(double)pixelCt);
                    int p2 =  (int)(angle2*(double)pixelCt);
//                    printf(" pixels1: %d,%d\n",p1,p2);
                    
                    // move "front" to the the middle and mod
                    p1 = (p1+pixelCt/2)%pixelCt;
                    p2 = (p2+pixelCt/2)%pixelCt;
                    
                    // work out the shortest route
                    int start,end;
                    if(p1>p2)std::swap(p1,p2);
                    if(p2-p1 > (p1+pixelCt)-p2){
                        // go through zero
                        start = p2;
                        end = p1+pixelCt;
                    } else {
                        start = p1;
                        end = p2;
                    }
                    
                    // add the pixels into the accumulator.
                    for(int i=start;i<=end;i++){
                        int j = i%pixelCt;
                        blurbuf[j*3+0]+=r;
                        blurbuf[j*3+1]+=g;
                        blurbuf[j*3+2]+=b;
                    }
                        
                }
            }
            
            // get the brightness, to which the values should sum
            double norm = getBright(distance)*255;
            
            // blur added 10/10/16
            blurnorm(pixelStore,blurbuf,norm,0,pixelCt);
            blurnorm(pixelStore,blurbuf,norm,1,pixelCt);
            blurnorm(pixelStore,blurbuf,norm,2,pixelCt);
            
            // construct the final data and publish
            data.pixels.clear();
            for(int i=0;i<pixelCt;i++){
/*                printf("Pix %d %f %f %f\n",
                       i,
                       pixelStore[i*3+0],
                       pixelStore[i*3+1],
                       pixelStore[i*3+2]);
 */
                p.r=std::min(255.0,pixelStore[i*3+0]);
                p.g=std::min(255.0,pixelStore[i*3+1]);
                p.b=std::min(255.0,pixelStore[i*3+2]);
                data.pixels.push_back(p);
            }
            pub.publish(data);
            updateOK=false;
        }
    }

private:
    int pixelCt;
    double *pixelStore,*blurbuf;
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
