#include "behaviortree_cpp_v3/bt_factory.h"
#include <ros/ros.h>

// file that contains the custom nodes definitions
#include "bt_test/dummy_nodes.h"
#include "bt_test/topic_logger.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bt_node");
    ros::NodeHandle n("~");
    std::string tree_path;
    n.getParam("tree_path", tree_path);

    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;

    // Note: the name used to register should be the same used in the XML.
    using namespace DummyNodes;

    // The recommended way to create a Node is through inheritance.
    factory.registerNodeType<ApproachObject>("ApproachObject");

    // Registering a SimpleActionNode using a function pointer.
    // you may also use C++11 lambdas instead of std::bind
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

    //You can also create SimpleActionNodes using methods of a class
    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper",
                                 std::bind(&GripperInterface::open, &gripper));
    factory.registerSimpleAction("CloseGripper",
                                 std::bind(&GripperInterface::close, &gripper));

    // Trees are created at deployment-time (i.e. at run-time, but only
    // once at the beginning).

    // IMPORTANT: when the object "tree" goes out of scope, all the
    // TreeNodes are destroyed
    auto tree = factory.createTreeFromFile(tree_path);

    ros::Publisher feedback_pub = n.advertise<std_msgs::String>("/bt_feedback", 10);
    TopicLogger tl(feedback_pub, tree);

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickRoot();

    return 0;
}
