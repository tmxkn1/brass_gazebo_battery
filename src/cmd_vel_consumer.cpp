#include <thread>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Battery.hh"
#include "gazebo/physics/physics.hh"

#include "ROS_debugging.h"
#include "cmd_vel_consumer.hh"


#define CMD_VEL_CONSUMER_DEBUG

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(CmdVelConsumerPlugin);

CmdVelConsumerPlugin::CmdVelConsumerPlugin() : consumerId(-1)
{
}

CmdVelConsumerPlugin::~CmdVelConsumerPlugin()
{
    if (this->battery && this->consumerId !=-1)
        this->battery->RemoveConsumer(this->consumerId);
}

void CmdVelConsumerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    #ifdef CMD_VEL_CONSUMER_DEBUG
        gzdbg << "started loading cmd_vel consumer \n";
    #endif

    // check if the ros is up!
    if (!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, _sdf->Get<std::string>("ros_node"), ros::init_options::NoSigintHandler);
    }

    this->model = _model;
    this->world = _model->GetWorld();

    std::string linkName = _sdf->Get<std::string>("link_name");
    std::string batteryName = _sdf->Get<std::string>("battery_name");
    std::string cmdVelTopic = _sdf->Get<std::string>("cmd_vel_topic");
    this->powerLoadRate = _sdf->Get<double>("power_load_rate");
    this->consumerIdlePower = _sdf->Get<double>("consumer_idle_power");
    GZ_ASSERT(this->powerLoadRate >= 0, "consume_rate cannot be negative.");
    GZ_ASSERT(this->consumerIdlePower >= 0, "consume_constant cannot be negative.");

    this->link = _model->GetLink(linkName);
    GZ_ASSERT(this->link, "Cannot find a link with the specified link_name.");
    this->battery = this->link->Battery(batteryName);
    GZ_ASSERT(this->link, "Cannot find a battery in the link with the specified battery_name. Make sure the batter_name specified in the plugin can be found in the specified link.");

    this->consumerId = this->battery->AddConsumer();
    this->battery->SetPowerLoad(this->consumerId, this->consumerIdlePower);

    this->rosNode.reset(new ros::NodeHandle(_sdf->Get<std::string>("ros_node")));
    this->cmd_vel_power_pub = this->rosNode->advertise<std_msgs::Float64>("/mobile_base/commands/consumer/cmd_vel_power", 1);
    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
        "/" + this->model->GetName() + cmdVelTopic,
        1,
        boost::bind(&CmdVelConsumerPlugin::OnCmdVelMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->cmd_vel_sub = this->rosNode->subscribe(so);
    this->rosQueueThread = std::thread(std::bind(&CmdVelConsumerPlugin::QueueThread, this));

    #ifdef CMD_VEL_CONSUMER_DEBUG
        gzdbg << "cmd_vel consumer loaded \n";
    #endif

    ROS_GREEN_STREAM("cmd_vel consumer loaded");

}

void CmdVelConsumerPlugin::Init()
{
#ifdef CMD_VEL_CONSUMER_DEBUG
    gzdbg << "cmd_vel_consumer is initialized \n";
#endif
    ROS_GREEN_STREAM("cmd_vel_consumer is initialized");
}

void CmdVelConsumerPlugin::Reset()
{
#ifdef CMD_VEL_CONSUMER_DEBUG
    gzdbg << "cmd_vel_consumer is reset \n";
#endif
    ROS_GREEN_STREAM("cmd_vel_consumer is reset");
}

double CmdVelConsumerPlugin::CalculatePower(const geometry_msgs::Twist::ConstPtr &_msg)
{
    double linearSpeed = _msg->linear.x + _msg->linear.y + _msg->linear.z;
    double angularSpeed = _msg->angular.z;
    return 5;
}

void CmdVelConsumerPlugin::OnCmdVelMsg(const geometry_msgs::Twist::ConstPtr &_msg)
{
  double cmd_vel_power = CalculatePower(_msg);
  this->battery->SetPowerLoad(this->consumerId, cmd_vel_power);
  std_msgs::Float64 cmd_vel_power_msg;
  cmd_vel_power_msg.data = cmd_vel_power;
  lock.lock();
  this->cmd_vel_power_pub.publish(cmd_vel_power_msg);
  lock.unlock();
}

void CmdVelConsumerPlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}