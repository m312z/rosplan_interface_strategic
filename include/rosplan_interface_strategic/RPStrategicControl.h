#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#ifndef KCL_strategic_control
#define KCL_strategic_control

/**
 * This file defines the RPStrategicControl class.
 * RPStrategicControl is used to decompose the goal into subtasks and generate new mission objects.
 * Plans are generated to compute duration and resource use of tasks.
 */
namespace KCL_rosplan {

	class RPStrategicControl
	{

	private:

		ros::NodeHandle* node_handle;

		/* rosplan knowledge interface */
		ros::ServiceClient update_knowledge_client;
		ros::ServiceClient current_goals_client;
		ros::ServiceClient current_knowledge_client;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> goals;
		std::map< std::string, std::vector<rosplan_knowledge_msgs::KnowledgeItem> > missions;

		/* planning interface */
		ros::ServiceClient problem_client;
		ros::ServiceClient planning_client;
		ros::ServiceClient parsing_client;

		rosplan_dispatch_msgs::EsterelPlan last_plan;
		bool new_plan_recieved;
		diagnostic_msgs::KeyValue getEndPoint(std::vector<rosplan_dispatch_msgs::EsterelPlanNode> & node) const;
		int getMinTime(rosplan_dispatch_msgs::EsterelPlan& plan) const;

	public:

		/* constructor */
		RPStrategicControl(ros::NodeHandle &nh);

		/* plan topic callback */
		void planCallback(const rosplan_dispatch_msgs::EsterelPlan& msg);

		/* problem decomposition service method */
		bool decomposeProblem(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool getMissionGoals(rosplan_knowledge_msgs::GetAttributeService::Request &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);
	};
}
#endif
