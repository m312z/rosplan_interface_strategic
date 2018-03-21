#include "rosplan_interface_strategic/RPStrategicControl.h"
#include <iostream>

//EsterelPlanEdge.h
//EsterelPlanNode.h
//EsterelPlanDispatcher.h

/* The implementation of RPStrategicControl.h */
namespace KCL_rosplan {

	/* constructor */
	RPStrategicControl::RPStrategicControl(ros::NodeHandle &nh) {

		node_handle = &nh;

		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		current_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_goals");
		//added just to work out what's going on
		current_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");

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
	 * get goals for particular mission
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

		//add can_tac_goto fact
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updateSrv.request.knowledge.instance_type = "";
		updateSrv.request.knowledge.instance_name = "";
		updateSrv.request.knowledge.function_value = 0;
		updateSrv.request.knowledge.attribute_name = "can_tac_goto";
		updateSrv.request.knowledge.values.clear();
		update_knowledge_client.call(updateSrv);

		//find and save and remove TIL's
		rosplan_knowledge_msgs::GetAttributeService currentAttribService;
		current_knowledge_client.call(currentAttribService);
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> knowledgeBase(currentAttribService.response.attributes);
		std::vector<ros::Time> initialTimes(currentAttribService.response.initial_time);
		std::vector<std::pair <rosplan_knowledge_msgs::KnowledgeItem, ros::Time> > tils;
		//TEST
		// for(int i = 0; i < knowledgeBase.size(); ++i){
		// 	if(knowledgeBase[i].attribute_name == "material_at" && knowledgeBase[i].is_negative == 1){
		// 		std::cout<<knowledgeBase[i]<<std::endl;
		// 		//std::cout<<initialTimes[i]<<std::endl;
		// 	}
		// }
		//SAVING AND REMOVING
		for(int i = 0; i < knowledgeBase.size(); ++i){
			if(knowledgeBase[i].attribute_name == "material_at" && initialTimes[i] > ros::Time::now()){
				tils.push_back(std::make_pair(knowledgeBase[i], initialTimes[i]));
				// std::cout<<tils[tils.size() - 1].first.attribute_name<<std::endl;
				// updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
				// updateSrv.request.knowledge = knowledgeBase[i];
				// updateSrv.request.duration = initialTimes[i].toSec();
				// std::cout<<initialTimes[i].toSec()<<std::endl;
				// update_knowledge_client.call(updateSrv);
			}
		}
		//TEST
		// std::cout<<"THIRD HERE!!!"<<std::endl;
		// rosplan_knowledge_msgs::GetAttributeService newAttribService;
		// current_knowledge_client.call(newAttribService);
		// std::vector<rosplan_knowledge_msgs::KnowledgeItem> newKnowledgeBase = newAttribService.response.attributes;
		// std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator nkitr = newKnowledgeBase.begin();
		// for(; nkitr != newKnowledgeBase.end(); ++nkitr){
		// 	if(nkitr->attribute_name == "material_at" && nkitr->is_negative == 1){
		// 		std::cout<<*nkitr<<std::endl;
		// 	}
		// }

		// compute mission durations
		std::stringstream ss;
		std::vector<double> mission_durations;
		//the locations the mission starts at, and so where the robot has to be to start the mission - for mission_at
		std::vector<diagnostic_msgs::KeyValue> start_locations;
		//the location the robot will be after the mission has been
		std::vector<diagnostic_msgs::KeyValue> end_points;
		//the lower bound times
		std::vector< std::pair<rosplan_dispatch_msgs::EsterelPlanNode, double> > lower_bounds;
		//the upper bound times
		std::vector< std::pair<rosplan_dispatch_msgs::EsterelPlanNode, double> > upper_bounds;
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
			ros::Duration(1).sleep(); // sleep for a second
			planning_client.call(empty);
			ros::Duration(1).sleep(); // sleep for a second
			parsing_client.call(empty);
			ros::Duration(1).sleep(); // sleep for a second

			while(!new_plan_recieved && ros::ok()) ros::spinOnce();

			// compute plan duration
			double max_time = 0;
			std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::iterator nit = last_plan.nodes.begin();
			std::cout<<last_plan<<std::endl;
			for(; nit != last_plan.nodes.end(); nit++) {
				//problem that this will overshoot time of plan now - however, could be fine as we should do upper estimate
				double time = nit->action.dispatch_time + nit->action.duration;
				if(time > max_time) max_time = time;
				//get the only first non move or undock action
				if(start_locations.size() <= mission_durations.size() && !(nit->action.name == "goto_waypoint" || nit->action.name == "undock" || nit->action.name == "localise")){
					//loop through parameters of that action
					for(std::vector<diagnostic_msgs::KeyValue>::iterator i = nit->action.parameters.begin(); i != nit->action.parameters.end(); ++i){
						if(i->key == "wp"){
							std::cout<<"for mission "<<ss.str()<<": "<<i->value<<std::endl;
							//first one that is a waypoint parameter is where the mission has to start (at_mission)
							start_locations.push_back(*i);
						}
					}
				}
			}
			//get the lower bound times
			lower_bounds = getLowerBoundTimes(last_plan);
			//get the upper bound times
			upper_bounds = getUpperBoundTimes(last_plan);
			//get the end points
			end_points.push_back(getEndPoint(last_plan.nodes));

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

		//remove can_tac_goto fact
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updateSrv.request.knowledge.attribute_name = "can_tac_goto";
		updateSrv.request.knowledge.values.clear();
		update_knowledge_client.call(updateSrv);

		//add can_strat_goto fact
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updateSrv.request.knowledge.instance_type = "";
		updateSrv.request.knowledge.instance_name = "";
		updateSrv.request.knowledge.function_value = 0;
		updateSrv.request.knowledge.attribute_name = "can_strat_goto";
		updateSrv.request.knowledge.values.clear();
		update_knowledge_client.call(updateSrv);

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

			// mission start
			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			updateSrv.request.knowledge.instance_type = "";
			updateSrv.request.knowledge.instance_name = "";
			updateSrv.request.knowledge.function_value = 0;
			updateSrv.request.knowledge.attribute_name = "mission_at";
			updateSrv.request.knowledge.values.clear();
			diagnostic_msgs::KeyValue pair_at_mission;
			pair_at_mission.key = "m";
			pair_at_mission.value = ss.str();
			updateSrv.request.knowledge.values.push_back(pair_at_mission);
			//am changing it now that the start of a mission is at the end of the last one - to change back can delete the if else
			//decided to keep it the normal way as explained in notes
			//if(i == 0){
				updateSrv.request.knowledge.values.push_back(start_locations[i]);
			/*}
			else{
				updateSrv.request.knowledge.values.push_back(end_points[i - 1]);
			}*/
			update_knowledge_client.call(updateSrv);

			//end_point
			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			updateSrv.request.knowledge.instance_type = "";
			updateSrv.request.knowledge.instance_name = "";
			updateSrv.request.knowledge.function_value = 0;
			updateSrv.request.knowledge.attribute_name = "end_point";
			updateSrv.request.knowledge.values.clear();
			diagnostic_msgs::KeyValue pair_end_point;
			pair_end_point.key = "m";
			pair_end_point.value = ss.str();
			updateSrv.request.knowledge.values.push_back(pair_end_point);
			updateSrv.request.knowledge.values.push_back(end_points[i]);
			update_knowledge_client.call(updateSrv);
		}
		//checking all facts and current state of problem - just to error check
		// rosplan_knowledge_msgs::GetAttributeService currentAttribService;
		// current_knowledge_client.call(currentAttribService);
		// std::vector<rosplan_knowledge_msgs::KnowledgeItem> attribs = currentAttribService.response.attributes;
		// std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator aitr = attribs.begin();
		// for( ; aitr != attribs.end(); ++aitr){
		// 	std::cout<<*aitr<<std::endl;
		// }
	}

	diagnostic_msgs::KeyValue RPStrategicControl::getEndPoint(std::vector<rosplan_dispatch_msgs::EsterelPlanNode> & nodes) const{
		std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::reverse_iterator nrit = nodes.rbegin();
		for(; nrit != nodes.rend(); ++nrit){
			if(!(nrit->action.name == "goto_waypoint" || nrit->action.name == "dock")){
				for(std::vector<diagnostic_msgs::KeyValue>::iterator i = nrit->action.parameters.begin(); i != nrit->action.parameters.end(); ++i){
					if(i->key == "wp"){
						return *i;
					}
				}
			}
		}
		return nodes[nodes.size() - 1].action.parameters[0];
	}

	//find the lower bound times for all actions to be executed
	std::vector< std::pair<rosplan_dispatch_msgs::EsterelPlanNode, double> > RPStrategicControl::getLowerBoundTimes(rosplan_dispatch_msgs::EsterelPlan& plan) const{
		std::vector< std::pair<rosplan_dispatch_msgs::EsterelPlanNode, double> > newNodes;
		std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::iterator pnit = plan.nodes.begin();
		newNodes.push_back(std::make_pair(*pnit, 0));
		++pnit;
		for(; pnit != plan.nodes.end(); ++pnit){
			newNodes.push_back(std::make_pair(*pnit, 100000));
		}

		//loop around all nodes
		std::vector< std::pair<rosplan_dispatch_msgs::EsterelPlanNode, double> >::iterator nit = newNodes.begin();
		for(; nit != newNodes.end(); ++nit){
			//for each node loop around all outgoing edges
			std::vector<int>::iterator eit = nit->first.edges_out.begin();
			for(; eit != nit->first.edges_out.end(); ++eit){
				//get the outgoing edge from the plan
				rosplan_dispatch_msgs::EsterelPlanEdge tempEdge = plan.edges[*eit];
				//use the edges sink to get the node that the edge is going to - assuming each edge can only go to one node
				rosplan_dispatch_msgs::EsterelPlanNode tempNode = plan.nodes[tempEdge.sink_ids[0]];
				//if the distance to the new node is greater than the distance from the old node + negative edge weight
				if(newNodes[tempNode.node_id].second > nit->second - tempEdge.duration_lower_bound){
					//set new distance to old distance + negative edge weight
					newNodes[tempNode.node_id].second = nit->second - tempEdge.duration_lower_bound;
				}
			}
		}
		// std::vector< std::pair<rosplan_dispatch_msgs::EsterelPlanNode, double> >::iterator nnit = newNodes.begin();
		// int i = 0;
		// for(; nnit != newNodes.end(); ++nnit){
		// 	std::cout<<"\n"<<std::endl;
		// 	std::cout<<"Node "<<i<<":"<<std::endl;
		// 	std::cout<<nnit->first<<std::endl;
		// 	std::cout<<"duration: "<<nnit->second<<std::endl;
		// }
		return newNodes;
	}

	//find the upper bound times for all actions to be executed
	std::vector< std::pair<rosplan_dispatch_msgs::EsterelPlanNode, double> > RPStrategicControl::getUpperBoundTimes(rosplan_dispatch_msgs::EsterelPlan& plan) const{
		std::vector< std::pair<rosplan_dispatch_msgs::EsterelPlanNode, double> > newNodes;
		std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::iterator pnit = plan.nodes.begin();
		newNodes.push_back(std::make_pair(*pnit, 0));
		++pnit;
		for(; pnit != plan.nodes.end(); ++pnit){
			newNodes.push_back(std::make_pair(*pnit, 100000));
		}

		//loop around all nodes
		std::vector< std::pair<rosplan_dispatch_msgs::EsterelPlanNode, double> >::iterator nit = newNodes.begin();
		for(; nit != newNodes.end(); ++nit){
			//for each node loop around all outgoing edges
			std::vector<int>::iterator eit = nit->first.edges_out.begin();
			for(; eit != nit->first.edges_out.end(); ++eit){
				//get the outgoing edge from the plan
				rosplan_dispatch_msgs::EsterelPlanEdge tempEdge = plan.edges[*eit];
				//use the edges sink to get the node that the edge is going to - assuming each edge can only go to one node
				rosplan_dispatch_msgs::EsterelPlanNode tempNode = plan.nodes[tempEdge.sink_ids[0]];
				//if the distance to the new node is greater than the distance from the old node + negative edge weight
				if(newNodes[tempNode.node_id].second > nit->second - tempEdge.duration_upper_bound){
					//set new distance to old distance + negative edge weight
					newNodes[tempNode.node_id].second = nit->second - tempEdge.duration_upper_bound;
				}
			}
		}
		return newNodes;
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
