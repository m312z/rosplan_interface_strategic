#include "rosplan_interface_strategic/RPStrategicControl.h"

/* The implementation of RPStrategicControl.h */
namespace KCL_rosplan {

	/* constructor */
	RPStrategicControl::RPStrategicControl(ros::NodeHandle &nh) {

		node_handle = &nh;

		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		current_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_goals");

		// planning interface
		std::string probTopic = "/rosplan_problem_interface/problem_generation_server";
		std::string planTopic = "/rosplan_planner_interface/planning_server";
		std::string parsTopic = "/rosplan_parsing_interface/parse_plan";

		node_handle->getParam("problem_service_topic", probTopic);
		node_handle->getParam("planning_service_topic", planTopic);
		node_handle->getParam("parsing_service_topic", parsTopic);

		problem_client = nh.serviceClient<std_srvs::Empty>(probTopic);
		planning_client = nh.serviceClient<std_srvs::Empty>(planTopic);
		parsing_client = nh.serviceClient<std_srvs::Empty>(parsTopic);
	}

	void RPStrategicControl::planCallback(const rosplan_dispatch_msgs::EsterelPlan& msg) {
		last_plan = msg;
		new_plan_recieved = true;
	}

	/*----------*/
	/* missions */
	/*----------*/

	/**
	 * get goals for partiular mission
	 */
	bool RPStrategicControl::getMissionGoals(rosplan_knowledge_msgs::GetAttributeService::Request &req, rosplan_knowledge_msgs::GetAttributeService::Response &res) {

		std::string mission_name = req.predicate_name;
		if(missions.find(mission_name)!=missions.end()) {
			res.attributes = missions.find(mission_name)->second;
		}

	}

	/**
	 * mission generation service method
	 */
	bool RPStrategicControl::decomposeProblem(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

		goals.clear();

		ROS_INFO("KCL: (%s) Decomposing problem by subgoals.", ros::this_node::getName().c_str());

		// retrieve old goals and save
		rosplan_knowledge_msgs::GetAttributeService currentGoalSrv;
		if (!current_goals_client.call(currentGoalSrv)) {
			ROS_ERROR("KCL: (%s) Failed to call goal service.", ros::this_node::getName().c_str());
		} else {
			goals = currentGoalSrv.response.attributes;
		}

		// clear old goals
		rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updateSrv.request.knowledge.attribute_name = "";
		updateSrv.request.knowledge.values.clear();
		update_knowledge_client.call(updateSrv);

		// compute mission durations
		std::stringstream ss;
		std::vector<double> mission_durations;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator git = goals.begin();
		for(; git!=goals.end(); git++) {

			ss.str("");
			ss << "mission_" << mission_durations.size();

			// insert new goal
			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
			updateSrv.request.knowledge = *git;
			update_knowledge_client.call(updateSrv);

			// generate problem and plan
			ROS_INFO("KCL: (%s) Generating plan for %s.", ros::this_node::getName().c_str(), ss.str().c_str());
			new_plan_recieved = false;

			std_srvs::Empty empty;
			problem_client.call(empty);
			planning_client.call(empty);
			parsing_client.call(empty);

			while(!new_plan_recieved && ros::ok()) ros::spinOnce();

			// compute plan duration
			double max_time = 0;
			std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::iterator nit = last_plan.nodes.begin();
			for(; nit != last_plan.nodes.end(); nit++) {
				double time = nit->action.dispatch_time + nit->action.duration;
				if(time > max_time) max_time = time;
			}
			mission_durations.push_back(max_time);
			missions[ss.str()];
			missions[ss.str()].push_back(*git);

			// clear goals again
			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			updateSrv.request.knowledge.attribute_name = "";
			updateSrv.request.knowledge.values.clear();
			update_knowledge_client.call(updateSrv);
		}

		// add new mission goals
		ROS_INFO("KCL: (%s) Adding new mission goals.", ros::this_node::getName().c_str());

		for(int i=0; i<mission_durations.size(); i++) {

			ss.str("");
			ss << "mission_" << i;

			// mission instance
			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			updateSrv.request.knowledge.instance_type = "mission";
			updateSrv.request.knowledge.instance_name = ss.str();
			updateSrv.request.knowledge.values.clear();
			update_knowledge_client.call(updateSrv);

			// mission duration
			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
			updateSrv.request.knowledge.attribute_name = "mission_duration";
			updateSrv.request.knowledge.values.clear();
			diagnostic_msgs::KeyValue pair_mission;
			pair_mission.key = "m";
			pair_mission.value = ss.str();
			updateSrv.request.knowledge.values.push_back(pair_mission);
			updateSrv.request.knowledge.function_value = mission_durations[i];
			update_knowledge_client.call(updateSrv);

			// mission goal
			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			updateSrv.request.knowledge.attribute_name = "mission_complete";
			update_knowledge_client.call(updateSrv);
		}
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_strategic_control");
		ros::NodeHandle nh("~");

		// params
		std::string lps_topic;
		nh.param("complete_plan_topic", lps_topic, std::string("/rosplan_plan_dispatcher/dispatch_plan"));

		// create bidder
		KCL_rosplan::RPStrategicControl rpsc(nh);

		// listen
		ros::ServiceServer service1 = nh.advertiseService("decompose_problem", &KCL_rosplan::RPStrategicControl::decomposeProblem, &rpsc);
		ros::ServiceServer service2 = nh.advertiseService("get_mission_goals", &KCL_rosplan::RPStrategicControl::getMissionGoals, &rpsc);

		ros::Subscriber lps = nh.subscribe(lps_topic, 100, &KCL_rosplan::RPStrategicControl::planCallback, &rpsc);

		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());

		ros::spin();

		return 0;
	}
