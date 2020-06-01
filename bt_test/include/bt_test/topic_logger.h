#pragma once
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "behaviortree_cpp_v3/blackboard.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

class TopicLogger: public BT::StatusChangeLogger
{
  public:
    TopicLogger(ros::Publisher pub, const BT::Tree& tree);
    void callback(
      BT::Duration timestamp,
      const BT::TreeNode & node,
      BT::NodeStatus prev_status,
      BT::NodeStatus status) override;

    // virtual ~TopicLogger() override;
    virtual void flush() override;

  private:
    ros::Publisher pub_;
};

TopicLogger::TopicLogger(ros::Publisher pub, const BT::Tree& tree)
  : StatusChangeLogger(tree.rootNode()), pub_(pub)
  {

  }

void TopicLogger::callback(
  BT::Duration timestamp,
  const BT::TreeNode & node,
  BT::NodeStatus prev_status,
  BT::NodeStatus status)
{
    using namespace std::chrono;

    constexpr const char* whitespaces = "                         ";
    constexpr const size_t ws_count = 25;

    double since_epoch = duration<double>(timestamp).count();

    std_msgs::String msg;
    std::ostringstream string_msg;
    string_msg << "[" << std::to_string(since_epoch) << "]: " << node.name()
               << &whitespaces[std::min(ws_count, node.name().size())]
               << toStr(prev_status, false) << " -> "
               << toStr(status, false);

    msg.data = string_msg.str();
    pub_.publish(msg);

    //TODO: the code below crashes the tree. Make an issue in BT repo
    BT::Blackboard::Ptr bb(node.config().blackboard);
    if (!bb )
    {
      ROS_WARN("Getting keys");
      auto keys = bb->getKeys();
      ROS_WARN("GOT Keys");
    }

  return;
}

void TopicLogger::flush()
{
  return;
}
