#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "formation_planner/optimizer_interface.h"
#include <thread>
#include <ros/callback_queue.h> // Add this line
using namespace formation_planner;
namespace gazebo
{
  class DynamicLinesMover : public ModelPlugin
  {
  public: 
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Initialize ROS node
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "dynamic_lines_mover");
      this->nh.reset(new ros::NodeHandle("dynamic_lines_mover"));

      // Subscribe to the topic for dynamic line positions
      this->sub = this->nh->subscribe("line_positions", 1000, &DynamicLinesMover::OnRosMsg, this);

      // Start a ROS spinning thread
      this->rosQueueThread = std::thread(std::bind(&DynamicLinesMover::QueueThread, this));
    }

    // Handle incoming ROS message
    public: void OnRosMsg(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
      if (msg->data.size() != 6 * num_robot_)
      {
        ROS_ERROR("Expected %ld elements in the message, got %ld", 6 * num_robot_, msg->data.size());
        return;
      }

      // Update the position and orientation of each line
      for (int i = 0; i < num_robot_; ++i)
      {
        ignition::math::Vector3d start(msg->data[i * 6], msg->data[i * 6 + 1], msg->data[i * 6 + 2]);
        ignition::math::Vector3d end(msg->data[i * 6 + 3], msg->data[i * 6 + 4], msg->data[i * 6 + 5]);
        ignition::math::Vector3d dir = end - start;
        double length = dir.Length();
        ignition::math::Quaterniond rot;
        rot.From2Axes(ignition::math::Vector3d::UnitZ, dir.Normalize());

        std::string link_name = "line" + std::to_string(i + 1);
        auto link = this->model->GetLink(link_name + "_link");
        if (link) {
          auto linkSDF = link->GetSDF();
          link->SetWorldPose(ignition::math::Pose3d((start + end) / 2, rot));
          link->SetScale(ignition::math::Vector3d(1, 1, length));
          auto collisionElem = linkSDF->GetElement("collision");
          auto visualElem = linkSDF->GetElement("visual");
        
          if (collisionElem && visualElem) {
            auto collisionGeometry = collisionElem->GetElement("geometry")->GetElement("cylinder");
            collisionGeometry->GetElement("length")->Set(length);

            auto visualGeometry = visualElem->GetElement("geometry")->GetElement("cylinder");
            visualGeometry->GetElement("length")->Set(length);
          }
        }
      }
    }

    // ROS queue thread function
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->nh->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // ROS NodeHandle
    private: std::unique_ptr<ros::NodeHandle> nh;

    // ROS subscriber
    private: ros::Subscriber sub;

    // ROS callback queue
    private: ros::CallbackQueue rosQueue;

    // ROS queue thread
    private: std::thread rosQueueThread;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DynamicLinesMover)
}
