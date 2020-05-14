#include <iostream>

#include <gazebo/common/Plugin.hh>
#include "ros/ros.h"

namespace gazebo
{
class HelloWorldPlugin : public WorldPlugin
{
public:
    HelloWorldPlugin() : WorldPlugin()
    {
        std::cout << "Hey!" << "\n";
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");

            return;
        }

        ROS_INFO("Hellow World!");
    }    
};

GZ_REGISTER_WORLD_PLUGIN(HelloWorldPlugin)

}