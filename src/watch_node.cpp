/**
 * @file watch_node.cpp
 * @brief Display sonar and light sensor data on a pioneer
 * body. Uses the C++ wheelyPioneer with a faked up robot
 * at a fixed position.
 *
 */

#include "ros/ros.h"

#include "lightsensor_gazebo/LightSensor.h"
#include "sensor_msgs/Range.h"

#define NUM_SONARS 8

#include "renderPioneer.h"

SDL sdl(600,600);
SDLContext context(&sdl);
World world;
Wheely robot(0,0,0,16);
PioneerRenderer rRobot(&context);

void sonarCallback(const sensor_msgs::Range::ConstPtr& msg,int i){
    robot.sonarDists[i] = ((double)msg->range)/1000.0;
}

uint8_t pix[1000][3];
int numPixels =0;

void lightCallback(const lightsensor_gazebo::LightSensor::ConstPtr& msg){
    numPixels = msg->pixels.size();
    for(int i=0;i<numPixels;i++){
        pix[i][0] = msg->pixels[i].r;
        pix[i][1] = msg->pixels[i].g;
        pix[i][2] = msg->pixels[i].b;
    }
}

void renderLightSensor(){
    double step = (2.0*PI)/(double)numPixels;
    double angle =0;
    const double radius = 2;
    for(int i=0;i<numPixels;i++){
        double y = sin(angle)*radius;
        double x = -cos(angle)*radius;
        
        uint32_t col = 255;
        col |= pix[i][0]<<24;
        col |= pix[i][1]<<16;
        col |= pix[i][2]<<8;
        
        sdl.setOutline(0);
        sdl.setFill(col);
        context.circle(x,y,0.1);
        angle += step;
    }
        
}

int main(int argc,char *argv[]){
    ros::init(argc,argv,"watch_node");
    ros::NodeHandle n;
    
    ros::Subscriber s[NUM_SONARS];
    for(int i=0;i<NUM_SONARS;i++){
        char buf[32];
        sprintf(buf,"s%d",i);
        s[i] = n.subscribe<sensor_msgs::Range>(buf,1000,
                                 boost::bind(sonarCallback,_1,i));
    }
    
    ros::Subscriber ls;
    ls = n.subscribe<lightsensor_gazebo::LightSensor>("light",1000,
                                                      lightCallback);
    
    // make the context view a world centered at zero
    Matrix3x3 worldmat;
    //    worldmat.makeScale(50,50);
    worldmat.scale(100,-100); // flip Y
    worldmat.rotate(PI*0.5);
    worldmat.translate(300,300);
    context.mult(worldmat);
    worldmat.dump();
    
    world.add(&robot,&rRobot);
    
    ros::Rate rate(10);
    while(ros::ok()){
        sdl.beginFrame();
        world.render();
        if(numPixels)renderLightSensor();
        if(sdl.endFrame()<0)break;
        ros::spinOnce();
        rate.sleep();
    }

}



