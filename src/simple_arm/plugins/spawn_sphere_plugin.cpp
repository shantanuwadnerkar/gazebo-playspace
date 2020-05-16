#include <iostream>
#include <thread>
#include <chrono>

#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
// #include <gazebo/common/Plugin.hh>



#include"ros/ros.h"

namespace gazebo
{
class SpawnSpherePlugin : public WorldPlugin
{
public:
    SpawnSpherePlugin() : WorldPlugin()
            {
                ROS_INFO("Spawn sphere plugin initialized!");
            }

    // ros::init("")
    // ros::ServiceServer srv_handle = 
    
    double totalSimulationTime{10};
    double filamentPerSimulator{1};
        

    void Load(physics::WorldPtr _parent, sdf::ElementPtr /*sdf*/)
    {
        // Option 1: Insert model from file via function call.
        // The filename must be in the GAZEBO_MODEL_PATH environment variable.
        // _parent->InsertModelFile("model://sphere");

        
        sdf::SDF sphereSDF;
        sphereSDF.SetFromString(
        "<sdf version ='1.6'>\
            <model name ='sphere'>\
            <static>false</static>\
                <pose>0 0 3 0 0 0</pose>\
                <link name ='link'>\
                    <self_collide>false</self_collide>\
                    <pose>0 0 0 0 0 0</pose>\
                    <collision name='collision'>\
                    <geometry>\
                    <sphere><radius>0.1</radius></sphere>\
                    </geometry>\
                    <surface>\
                        <contact><collide_bitmask>0x01</collide_bitmask></contact>\
                    </surface>\
                    </collision>\
                    <visual name ='visual'>\
                    <geometry>\
                    <sphere><radius>0.01</radius></sphere>\
                    </geometry>\
                    </visual>\
                </link>\
            </model>\
        </sdf>");
        
        int i{0};


        std::vector<sdf::ElementPtr> modelVector;
        // resize vec here with totalSimulationTime * filamentPerSimulator

        for (0; i<totalSimulationTime*filamentPerSimulator; i++)
        {
            std::stringstream ssVarName;
            std::stringstream ssHex;
            ssVarName << "filament_" << i;
            std::string varName = ssVarName.str();

            sdf::ElementPtr modelsdfPtr = sphereSDF.Root();
            modelsdfPtr->GetElement("model")->GetAttribute("name")->SetFromString(varName);
            
            ssHex << "0x" << std::hex << i+1;
            std::cout << ssHex.str();
            sdf::ElementPtr collisionPtr = modelsdfPtr->GetElement("model")->GetElement("link")->GetElement("collision");
            sdf::ElementPtr contactPtr = collisionPtr->GetElement("surface")->GetElement("contact");
            sdf::ParamPtr collideBitmaskPtr = contactPtr->GetElement("collide_bitmask")->GetValue();
            bool setCollideBitmask = contactPtr->GetElement("collide_bitmask")->Set(ssHex.str());

            modelVector.push_back(modelsdfPtr);
            _parent->InsertModelSDF(sphereSDF);


            // std::cout << *collideBitmaskPtr << std::endl;
            // std::cout << setCollideBitmask << std::endl;
            



            // sdf::ElementPtr posePtr = sphereSDF.Root()->GetElement("model")->GetElement("pose");
            // // std::cout << posePtr->Get<ignition::math::Pose3d>() << std::endl;

            // sdf::ElementPtr posePtr;
            // sphereSDF.Root()->AddElementDescription(posePtr);
            // posePtr->AddValue("ignition::math::Pose3d", "0.0, 0.0, 0.0, 0.0, 0.0, 0.0", 1,"Pose of the model");

            // AddValue("ignition::math::Pose3d", "0.0, 0.0, 0.0, 0.0, 0.0, 0.0", 1,"Pose of the model");

            // sdf::ElementPtr posePtr = sphereSDF.Root()->AddElement("pose");
            // posePtr->AddValue("ignition::math::Pose3d", "0.0, 0.0, 0.0, 0.0, 0.0, 0.0", 1,"Pose of the model");

            // sdf::ParamPtr val = posePtr->GetValue();

            // ignition::math::Vector3d sdfPose = sphereSDF.Root()->GetElement("pose")->Get<ignition::math::Pose3d>().Pos();
            // ignition::math::Pose3d modelBool = sphereSDF.Root()->Get<ignition::math::Pose3d>();

            // std::size_t sizet = modelPtr->GetAttributeCount();
            // std::cout << posePtr << std::endl;

            // sdf::Pose pose = sdf::Pose();
            // posePtr->SetAttribute(sdf::Pose::Pose(0.0, 0.0, 5.0, 0.0, 0.0, 0.0));

        }

        // sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
        // model->GetAttribute("name")->SetFromString("unique_sphere");
        // _parent->InsertModelSDF(sphereSDF);
        // std::string sdf_string = sphereSDF.ToString();

        ROS_INFO("Sphere initializing...");

        for (std::vector<sdf::ElementPtr>::const_iterator i = modelVector.begin(); i != modelVector.end(); ++i)
            std::cout << *i << ' ';

        ROS_INFO("Sphere initialized!");
        std::chrono::milliseconds timespan(2000); // or whatever
        std::this_thread::sleep_for(timespan);
        ROS_INFO("Sleep over!");



    }
};

GZ_REGISTER_WORLD_PLUGIN(SpawnSpherePlugin)

}


























































// #include <iostream>
// #include "gazebo/gazebo.hh"
// #include "gazebo/physics/physics.hh"
// #include "gazebo/common/Plugin.hh"
// #include "gazebo/transport/transport.hh"
// #include <math.h>

// namespace gazebo
// {
//   class FactoryLiquid : public WorldPlugin
//   {
//         public: virtual ~FactoryLiquid()
//         {

//         }
//     public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
//     {
//         math::Vector3 p3, init_pos;

//         ////////////////////////////////////////////////////////////////
//         /////// SDF PARAMETERS

//         ////////////// Get nr of spheres
//         if (!_sdf->HasElement("nr_spheres"))
//         {
//           std::cout << "Missing parameter <nr_spheres> in FactoryLiquid, default to 0" << std::endl;
//           nr_spheres = 0;
//         }
//         else nr_spheres = _sdf->GetElement("nr_spheres")->GetValueUInt();

//         ////////////// Set up the initial position parameter
//         if (!_sdf->HasElement("init_pos"))
//         {
//           std::cout << "Missing parameter <init_pos> in FactoryLiquid, default to 0 0 0" << std::endl;
//           init_pos.x = 0.0;
//           init_pos.y = 0.0;
//           init_pos.z = 0.0;
//         }
//         else init_pos = _sdf->GetElement("init_pos")->GetValueVector3();

//         // etc. other parameters from the world file

//         /////// END SDF PARAMETERS
//         //////////////////////////////////////////////////////////////////


//         //////////////////////////////////////////////////////////////////
//         //////////////////////////////////////////////////////////////////
//         //////////////////////////// START XML LIQUID
//         xml << "<?xml version='1.0'?>\n";
//         xml << "<sdf version='1.4'>\n";
//         xml << "<model name='liquid_spheres'>\n";
//         xml << "\t<static>false</static>\n";
//         xml << "\t<pose>" << init_pos.x << " " << init_pos.y << " " << init_pos.z << " 0 0 0 </pose>\n";

//         for (unsigned int i=0; i<nr_spheres; i++)
//         {
//                 p3 = FactoryLiquid::part_position(i, radius, spawned, level);
//                 xml << "\t\t<link name='sphere_link_" << i << "'>\n";
//                 xml << "\t\t\t<self_collide>true</self_collide>\n";
//                 xml << "\t\t\t<pose>" << p3.x << " " << p3.y << " " << p3.z << " 0 0 0</pose>\n";

//                 xml << "\t\t\t<inertial>\n";
//                 xml << "\t\t\t\t<pose> 0 0 0 0 0 0 </pose>\n";
//                 xml << "\t\t\t\t<inertia>\n";
//                 xml << "\t\t\t\t\t<ixx>" << inertia << "</ixx>\n";
//                 xml << "\t\t\t\t\t<ixy>0</ixy>\n";
//                 xml << "\t\t\t\t\t<ixz>0</ixz>\n";
//                 xml << "\t\t\t\t\t<iyy>" << inertia << "</iyy>\n";
//                 xml << "\t\t\t\t\t<iyz>0</iyz>\n";
//                 xml << "\t\t\t\t\t<izz>" << inertia << "</izz>\n";
//                 xml << "\t\t\t\t</inertia>\n";
//                 xml << "\t\t\t\t<mass>" << mass << "</mass>\n";
//                 xml << "\t\t\t</inertial>\n";

//                 xml << "\t\t\t<collision name='collision_" << i << "'>\n";
//                 xml << "\t\t\t\t<geometry>\n";
//                 xml << "\t\t\t\t\t<sphere>\n";
//                 xml << "\t\t\t\t\t\t<radius>" << radius << "</radius>\n";
//                 xml << "\t\t\t\t\t</sphere>\n";
//                 xml << "\t\t\t\t</geometry>\n";
//                 xml << "\t\t\t\t<surface>\n";
//                 xml << "\t\t\t\t\t<friction>\n";
//                 xml << "\t\t\t\t\t\t<ode>\n";
//                 xml << "\t\t\t\t\t\t\t<mu>" << mu << "</mu>\n";
//                 xml << "\t\t\t\t\t\t\t<mu2>" << mu2 << "</mu2>\n";
//                 xml << "\t\t\t\t\t\t\t<fdir1>0.0 0.0 0.0</fdir1>\n";
//                 xml << "\t\t\t\t\t\t\t<slip1>" << slip1 << "</slip1>\n";
//                 xml << "\t\t\t\t\t\t\t<slip2>" << slip2 << "</slip2>\n";
//                 xml << "\t\t\t\t\t\t</ode>\n";
//                 xml << "\t\t\t\t\t</friction>\n";
//                 xml << "\t\t\t\t\t<bounce>\n";
//                 xml << "\t\t\t\t\t\t<restitution_coefficient>" << bounce << "</restitution_coefficient>\n";
//                 xml << "\t\t\t\t\t\t<threshold>10000.0</threshold>\n";
//                 xml << "\t\t\t\t\t</bounce>\n";
//                 xml << "\t\t\t\t\t<contact>\n";
//                 xml << "\t\t\t\t\t\t<ode>\n";
//                 xml << "\t\t\t\t\t\t\t<soft_cfm>" << cfm << "</soft_cfm>\n";
//                 xml << "\t\t\t\t\t\t\t<soft_erp>" << erp << "</soft_erp>\n";
//                 xml << "\t\t\t\t\t\t\t<kp>" << kp << "</kp>\n";
//                 xml << "\t\t\t\t\t\t\t<kd>" << kd << "</kd>\n";
//                 xml << "\t\t\t\t\t\t\t<max_vel>100.0</max_vel>\n";
//                 xml << "\t\t\t\t\t\t\t<min_depth>0.001</min_depth>\n";
//                 xml << "\t\t\t\t\t\t</ode>\n";
//                 xml << "\t\t\t\t\t</contact>\n";
//                 xml << "\t\t\t\t</surface>\n";
//                 xml << "\t\t\t</collision>\n";

//                 xml << "\t\t\t<visual name='sphere_visual_" << i << "'>\n";
//                 xml << "\t\t\t\t<geometry>\n";
//                 xml << "\t\t\t\t\t<sphere>\n";
//                 xml << "\t\t\t\t\t\t<radius>" << radius << "</radius>\n";
//                 xml << "\t\t\t\t\t</sphere>\n";
//                 xml << "\t\t\t\t</geometry>\n";
//                 xml << "\t\t\t\t<material>\n";
//                 xml << "\t\t\t\t\t<script>\n";
//                 xml << "\t\t\t\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n";
//                 xml << "\t\t\t\t\t\t<name>Gazebo/Red</name>\n";
//                 xml << "\t\t\t\t\t</script>\n";
//                 xml << "\t\t\t\t</material>\n";
//                 xml << "\t\t\t</visual>\n";
//                 xml << "\t\t</link>\n";
//         }

//                 xml << "</model>\n";
//                 xml << "</gazebo>\n";

//                 /////////////////////////////////////////
//         //std::cout << xml.str() << "\n";

//         sdf::SDF sphereSDF;
//         sphereSDF.SetFromString(xml.str());


//         _parent->InsertModelSDF(sphereSDF);

//     }

//     // function to position the particles
//     public: math::Vector3 part_position(int i, double radius, int& spawned, int& level)
//     {

//                 return v3;
//     }
// };

//   // Register this plugin with the simulator
//   GZ_REGISTER_WORLD_PLUGIN(FactoryLiquid)
// }
