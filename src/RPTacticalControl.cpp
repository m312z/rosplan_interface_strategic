#include "rosplan_interface_strategic/RPTacticalControl.h"

/* The implementation of RPTacticalControl.h */
namespace KCL_rosplan {

    /* constructor */
    RPTacticalControl::RPTacticalControl(ros::NodeHandle &nh) {

        current_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/goals");
        local_update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/rosplan_knowledge_base/update");

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
        dispatch_client = nh.serviceClient<rosplan_dispatch_msgs::DispatchService>(dispTopic);

        nh.getParam("robot_name", robot_name);
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
        local_update_knowledge_client.call(updateSrv);

        // add mission goal to knowledge base
        rosplan_knowledge_msgs::KnowledgeUpdateService updateGoalSrv;
        updateGoalSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
        updateGoalSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
        for(int i = 0; i<mission_goals.size(); i++) {
            updateGoalSrv.request.knowledge = mission_goals[i];
            if(!local_update_knowledge_client.call(updateGoalSrv)) {
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
            if(!local_update_knowledge_client.call(updateGoalSrv)) {
                ROS_INFO("KCL: (%s) failed to update PDDL goal.", ros::this_node::getName().c_str());
            }
        }

        // add old goal to knowledge base
        updateGoalSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
        updateGoalSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
        for(int i = 0; i<old_goals.size(); i++) {
            updateGoalSrv.request.knowledge = old_goals[i];
            if(!local_update_knowledge_client.call(updateGoalSrv)) {
                ROS_INFO("KCL: (%s) failed to update PDDL goal.", ros::this_node::getName().c_str());
            }
        }
    }

    /* action dispatch callback */
    bool RPTacticalControl::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        // get mission ID and robot from action dispatch complete_mission (?r - robot ?m - mission ?wp - waypoint)
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
        rosplan_dispatch_msgs::DispatchService dispatch;
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
            bool dispatch_success = dispatch_client.call(dispatch);

            restoreGoals();
            return dispatch_success;
        }

        restoreGoals();
        return false;
    }

	/* run action interface */
	void RPTacticalControl::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        // check robot
        for(size_t i=0; i<msg->parameters.size(); i++) {
            if(0==msg->parameters[i].key.compare("r")
                    && 0!=msg->parameters[i].value.compare(RPTacticalControl::robot_name)) {
                ROS_INFO("KCL: (%s) ignoring action for other robot.", params.name.c_str());
                return;
            }
        }

        RPActionInterface::dispatchCallback(msg);
	}

	/* run action interface */
	void RPTacticalControl::runActionInterface() {

		ros::NodeHandle nh("~");

		// set action name
		nh.getParam("pddl_action_name", params.name);

		// knowledge base services
		std::string kb = "knowledge_base";
		nh.getParam("knowledge_base", kb);

		// fetch action params
		std::stringstream ss;
		ss << "/" << kb << "/domain/operator_details";
		ros::service::waitForService(ss.str(),ros::Duration(20));
		ros::ServiceClient client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>(ss.str());
		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = params.name;
		if(client.call(srv)) {
			params = srv.response.op.formula;
			op = srv.response.op;
		} else {
			ROS_ERROR("KCL: (RPActionInterface) could not call Knowledge Base for operator details, %s", params.name.c_str());
			return;
		}

		// collect predicates from operator description
		std::vector<std::string> predicateNames;

		// effects
		std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator pit = op.at_start_add_effects.begin();
		for(; pit!=op.at_start_add_effects.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.at_start_del_effects.begin();
		for(; pit!=op.at_start_del_effects.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.at_end_add_effects.begin();
		for(; pit!=op.at_end_add_effects.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.at_end_del_effects.begin();
		for(; pit!=op.at_end_del_effects.end(); pit++)
			predicateNames.push_back(pit->name);

		// simple conditions
		pit = op.at_start_simple_condition.begin();
		for(; pit!=op.at_start_simple_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.over_all_simple_condition.begin();
		for(; pit!=op.over_all_simple_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.at_end_simple_condition.begin();
		for(; pit!=op.at_end_simple_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		// negative conditions
		pit = op.at_start_neg_condition.begin();
		for(; pit!=op.at_start_neg_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.over_all_neg_condition.begin();
		for(; pit!=op.over_all_neg_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.at_end_neg_condition.begin();
		for(; pit!=op.at_end_neg_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		// fetch and store predicate details
		ss.str("");
		ss << "/" << kb << "/domain/predicate_details";
		ros::service::waitForService(ss.str(),ros::Duration(20));
		ros::ServiceClient predClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainPredicateDetailsService>(ss.str());
		std::vector<std::string>::iterator nit = predicateNames.begin();
		for(; nit!=predicateNames.end(); nit++) {
			if (predicates.find(*nit) != predicates.end()) continue;
			if (*nit == "=" || *nit == ">" || *nit == "<" || *nit == ">=" || *nit == "<=") continue;
			rosplan_knowledge_msgs::GetDomainPredicateDetailsService predSrv;
			predSrv.request.name = *nit;
			if(predClient.call(predSrv)) {
				if (predSrv.response.is_sensed){
					sensed_predicates.insert(std::pair<std::string, rosplan_knowledge_msgs::DomainFormula>(*nit, predSrv.response.predicate));	
				} else {
					predicates.insert(std::pair<std::string, rosplan_knowledge_msgs::DomainFormula>(*nit, predSrv.response.predicate));	
				}
			} else {
				ROS_ERROR("KCL: (RPActionInterface) could not call Knowledge Base for predicate details, %s", params.name.c_str());
				return;
			}
		}

		// create PDDL info publisher
		ss.str("");
		ss << "/" << kb << "/pddl_action_parameters";
		pddl_action_parameters_pub = nh.advertise<rosplan_knowledge_msgs::DomainFormula>(ss.str(), 10, true);

		// create the action feedback publisher
		std::string aft = "default_feedback_topic";
		nh.getParam("action_feedback_topic", aft);
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>(aft, 10, true);

		// knowledge interface
		ss.str("");
		ss << "/" << kb << "/update_array";
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>(ss.str());

		// listen for action dispatch
		std::string adt = "default_dispatch_topic";
		nh.getParam("action_dispatch_topic", adt);

        ros::SubscribeOptions ops;
        ops.template init<rosplan_dispatch_msgs::ActionDispatch>(adt, 1000, boost::bind(&KCL_rosplan::RPTacticalControl::dispatchCallback, this, _1));
        ops.transport_hints = ros::TransportHints();
        ops.allow_concurrent_callbacks = true;
		ros::Subscriber ds = nh.subscribe(ops);

		// loop
		ros::Rate loopRate(1);
		ros::AsyncSpinner spinner(4);
        spinner.start();

		ROS_INFO("KCL: (%s) Ready to receive", params.name.c_str());

		while(ros::ok()) {
			pddl_action_parameters_pub.publish(params);
			loopRate.sleep();
		}
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
