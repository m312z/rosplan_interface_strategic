#include "rosplan_interface_strategic/RPTacticalControl.h"

/* The implementation of RPTacticalControl.h */
namespace KCL_rosplan {

	/* constructor */
	RPTacticalControl::RPTacticalControl(ros::NodeHandle &nh) {

		// planning interface
		std::string cancTopic = "/rosplan_plan_dispatcher/cancel_dispatch";
		std::string probTopic = "/rosplan_problem_interface/problem_generation_server";
		std::string planTopic = "/rosplan_planner_interface/planning_server";
		std::string parsTopic = "/rosplan_parsing_interface/parse_plan";
		std::string dispTopic = "/rosplan_plan_dispatch/dispatch_plan";

		nh.getParam("cancel_service_topic", cancTopic);
		nh.getParam("problem_service_topic", probTopic);
		nh.getParam("planning_service_topic", planTopic);
		nh.getParam("parsing_service_topic", parsTopic);
		nh.getParam("dispatch_service_topic", dispTopic);

		cancel_client = nh.serviceClient<std_srvs::Empty>(cancTopic);
		problem_client = nh.serviceClient<std_srvs::Empty>(probTopic);
		planning_client = nh.serviceClient<std_srvs::Empty>(planTopic);
		parsing_client = nh.serviceClient<std_srvs::Empty>(parsTopic);
		dispatch_client = nh.serviceClient<std_srvs::Empty>(dispTopic);
	}

	/* action dispatch callback */
	bool RPTacticalControl::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// get mission ID from action dispatch (?r - robot ?i - item ?wp - waypoint)
		std::string item;
		std::string wp;
		bool found_item = false;
		bool found_wp = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("i")) {
				item = msg->parameters[i].value;
				found_item = true;
			}
			if(0==msg->parameters[i].key.compare("wp")) {
				wp = msg->parameters[i].value;
				found_wp = true;
			}
		}
		if(!found_item || !found_wp) {
			ROS_INFO("KCL: (%s) aborting action dispatch; PDDL action missing required parameter ?m", params.name.c_str());
			return false;
		}

		// add goal to knowledge base
		rosplan_knowledge_msgs::KnowledgeUpdateService updateGoalSrv;
		updateGoalSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		updateGoalSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updateGoalSrv.request.knowledge.attribute_name = "complete";
		updateGoalSrv.request.knowledge.values.clear();
		diagnostic_msgs::KeyValue pair_item;
		pair_item.key = "i";
		pair_item.value = item;
		updateGoalSrv.request.knowledge.values.push_back(pair_item);
		diagnostic_msgs::KeyValue pair_wp;
		pair_wp.key = "wp";
		pair_wp.value = wp;
		updateGoalSrv.request.knowledge.values.push_back(pair_wp);
		if(!update_knowledge_client.call(updateGoalSrv)) {
			ROS_INFO("KCL: (%s) failed to update PDDL goal.", ros::this_node::getName().c_str());
		}

		// remove stragic planning flag from knowledge base
		rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
		updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
		updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updatePredSrv.request.knowledge.attribute_name = "strategic_planning";
		updatePredSrv.request.knowledge.values.clear();
		if(!update_knowledge_client.call(updatePredSrv)) {
			ROS_INFO("KCL: (%s) failed to remove strategic_planning fact.", ros::this_node::getName().c_str());
		}

		// generate problem and plan
		ROS_INFO("KCL: (%s) Sending to planning system.", ros::this_node::getName().c_str());

		std_srvs::Empty empty;
		cancel_client.call(empty);
		problem_client.call(empty);
		planning_client.call(empty);
		parsing_client.call(empty);

		bool dispatch_success = dispatch_client.call(empty);

		// remove goal from KB
		updateGoalSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
		if(!update_knowledge_client.call(updateGoalSrv)) {
			ROS_INFO("KCL: (%s) failed to update PDDL goal.", ros::this_node::getName().c_str());
		}

		// add strategic planning flag to KB
		updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		if(!update_knowledge_client.call(updatePredSrv)) {
			ROS_INFO("KCL: (%s) failed to add strategic_planning fact.", ros::this_node::getName().c_str());
		}

		return dispatch_success;
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
