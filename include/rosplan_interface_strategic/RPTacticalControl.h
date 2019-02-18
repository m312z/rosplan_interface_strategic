#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <tf/transform_listener.h>

#include "rosplan_action_interface/RPActionInterface.h"

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/DispatchService.h"

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#ifndef KCL_tactical_control
#define KCL_tactical_control

/**
 * This file defines the RPTacticalControl class.
 * RPTacticalControl encapsulates as an action:
 * problem generation, planning, and dispatch.
 */
namespace KCL_rosplan {

    class RPTacticalControl: public RPActionInterface
    {

    private:

        ros::ServiceClient current_goals_client;
        ros::ServiceClient mission_goals_client;
        ros::ServiceClient cancel_client;
        ros::ServiceClient problem_client;
        ros::ServiceClient planning_client;
        ros::ServiceClient parsing_client;
        ros::ServiceClient dispatch_client;

        ros::ServiceClient local_update_knowledge_client;

        std::vector<rosplan_knowledge_msgs::KnowledgeItem> mission_goals;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> old_goals;

    public:

        /* constructor */
        RPTacticalControl(ros::NodeHandle &nh);

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
        bool initGoals(const std::string &mission);
        void restoreGoals();
    };
}
#endif