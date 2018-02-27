#include "rosplan_interface_strategic/RPTacticalControl.h"

/* The implementation of RPTacticalControl.h */
namespace KCL_rosplan {

	/* constructor */
	RPTacticalControl::RPTacticalControl(ros::NodeHandle &nh) {

		current_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_goals");

		// planning interface
		std::string goalTopic = "/rosplan_interface_strategic_control/get_mission_goals";
		std::string cancTopic = "/rosplan_plan_dispatcher/cancel_dispatch";
		std::string probTopic = "/rosplan_problem_interface/problem_generation_server";
		std::string planTopic = "/rosplan_planner_interface/planning_server";
		std::string parsTopic = "/rosplan_parsing_interface/parse_plan";
		std::string dispTopic = "/rosplan_plan_dispatch/dispatch_plan";

		nh.getParam("mission_goals_topic", goalTopic);
		nh.getParam("cancel_service_topic", cancTopic);
		nh.getParam("problem_service_topic", probTopic);
		nh.getParam("planning_service_topic", planTopic);
		nh.getParam("parsing_service_topic", parsTopic);
		nh.getParam("dispatch_service_topic", dispTopic);

		mission_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(goalTopic);
		cancel_client = nh.serviceClient<std_srvs::Empty>(cancTopic);
		problem_client = nh.serviceClient<std_srvs::Empty>(probTopic);
		planning_client = nh.serviceClient<std_srvs::Empty>(planTopic);
		parsing_client = nh.serviceClient<std_srvs::Empty>(parsTopic);
		dispatch_client = nh.serviceClient<std_srvs::Empty>(dispTopic);
	}

	/**
	 * fetch goals for corresponding mission and update KB
	 */
	bool RPTacticalControl::initGoals(const std::string &mission) {

		mission_goals.clear();
		old_goals.clear();

		// fetch mission goals
		rosplan_knowledge_msgs::GetAttributeService gsrv;
		gsrv.request.predicate_name = mission;
		if(!mission_goals_client.call(gsrv)) {
			ROS_ERROR("KCL: (%s) Failed to call Knowledge Base for goals.", ros::this_node::getName().c_str());
			return false;
		} else {
			mission_goals = gsrv.response.attributes;
		}

		// fetch and store old goals
		rosplan_knowledge_msgs::GetAttributeService currentGoalSrv;
		if (!current_goals_client.call(currentGoalSrv)) {
			ROS_ERROR("KCL: (%s) Failed to call Knowledge Base for goals.", ros::this_node::getName().c_str());
			return false;
		} else {
			old_goals = currentGoalSrv.response.attributes;
		}

		// clear old goals
		rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updateSrv.request.knowledge.attribute_name = "";
		updateSrv.request.knowledge.values.clear();
		update_knowledge_client.call(updateSrv);

		// add mission goal to knowledge base
		rosplan_knowledge_msgs::KnowledgeUpdateService updateGoalSrv;
		updateGoalSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		updateGoalSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		for(int i = 0; i<mission_goals.size(); i++) {
			updateGoalSrv.request.knowledge = mission_goals[i];
			if(!update_knowledge_client.call(updateGoalSrv)) {
				ROS_INFO("KCL: (%s) failed to update PDDL goal.", ros::this_node::getName().c_str());
				restoreGoals();
				return false;
			}
		}

		return true;

	}

	/**
	 * remove mission goals from KB and restore saved goals
	 */
	void RPTacticalControl::restoreGoals() {

		// remove mission goal from KB
		rosplan_knowledge_msgs::KnowledgeUpdateService updateGoalSrv;
		updateGoalSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
		for(int i = 0; i<mission_goals.size(); i++) {
			updateGoalSrv.request.knowledge = mission_goals[i];
			if(!update_knowledge_client.call(updateGoalSrv)) {
				ROS_INFO("KCL: (%s) failed to update PDDL goal.", ros::this_node::getName().c_str());
			}
		}

		// add old goal to knowledge base
		updateGoalSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		updateGoalSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		for(int i = 0; i<old_goals.size(); i++) {
			updateGoalSrv.request.knowledge = old_goals[i];
			if(!update_knowledge_client.call(updateGoalSrv)) {
				ROS_INFO("KCL: (%s) failed to update PDDL goal.", ros::this_node::getName().c_str());
			}
		}
	}

	/* action dispatch callback */
	bool RPTacticalControl::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// get mission ID from action dispatch complete_mission (?r - robot ?m - mission ?wp - waypoint)
		std::string mission;
		bool found_mission = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("m")) {
				mission = msg->parameters[i].value;
				found_mission = true;
			}
		}
		if(!found_mission) {
			ROS_INFO("KCL: (%s) aborting action dispatch; PDDL action missing required parameter ?m", params.name.c_str());
			return false;
		}

		if(!initGoals(mission)) return false;

		// generate problem and plan
		ROS_INFO("KCL: (%s) Sending to planning system.", ros::this_node::getName().c_str());

		std_srvs::Empty empty;
		cancel_client.call(empty);
		ros::Duration(1).sleep(); // sleep for a second
		problem_client.call(empty);
		ros::Duration(1).sleep(); // sleep for a second

		// send to planner
		if(planning_client.call(empty)) {
			ros::Duration(1).sleep(); // sleep for a second
			// parse planner output
			parsing_client.call(empty);
			ros::Duration(1).sleep(); // sleep for a second

			// dispatch tactical plan
			bool dispatch_success = dispatch_client.call(empty);

			restoreGoals();
			return dispatch_success;
		}

		restoreGoals();
		return false;
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_tactical_control");
		ros::NodeHandle nh("~");

		// create PDDL action subscriber
		KCL_rosplan::RPTacticalControl rptc(nh);

		rptc.runActionInterface();

		return 0;
	}
