#define NAME_OF_THIS_NODE "trajectoryPlanner"

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
//#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <quadrivio_msgs/PathWithVelocity.h>
#include <tf/tf.h>
#include "sbpl/headers.h"
#include <boost/concept_check.hpp>

class ROSnode {
private:
ros::NodeHandle Handle;
ros::Subscriber SubscriberG;
ros::Subscriber SubscriberS;
ros::Subscriber SubscriberM;
ros::Publisher Publisher;
ros::Publisher TestPublisher;
tf::Pose start;
double startv;
double startomega;
bool haveStart;
tf::Pose goal;
double goalv;
double goalomega;
bool haveGoal;
double * map;
int mapWidth;
int mapHeight;
bool haveMap;
std::vector<sbpl_2Dcell_t> changedCells;
unsigned long seq;
std::string primitivesFile;
std::string envType;
double startEpsilon;
double vehicleWidth;
double vehicleLength;
int obsthresh;
int cost_inscribed_thresh;
int cost_possibly_circumscribed_thresh;
int cellsize_m;
double nominalvel;
double timetoturn;
int numTheta;
int numV;
std::vector<double> velocities;
int numOmega;
std::vector<double> omegas;
int numSteers;
double allocated_time_secs_foreachplan;
bool bsearchuntilfirstsolution;
bool bforwardsearch;
void plannerGetGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
//void plannerGetStartCallback(const nav_msgs::Odometry::ConstPtr& start);
void plannerGetStartCallback(const geometry_msgs::PoseStamped::ConstPtr& start);
void plannerGetMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
public:
ROSnode();
bool Prepare();
int plan();
};

ROSnode::ROSnode(){
	map = NULL;
	haveStart = false;
	haveGoal = false;
	haveMap = false;
	changedCells.clear();
}

bool ROSnode::Prepare() {
	//Retrieve parameters
	if (Handle.getParam("/trajectoryPlanner/primitivesFile", primitivesFile)) {
		ROS_INFO("Node %s: retrieved parameter primitivesFile.", ros::this_node::getName().c_str());
	}
	else {
		ROS_FATAL("Node %s: unable to retrieve parameter primitivesFile.", ros::this_node::getName().c_str());
		return false;
	}
	
	if (Handle.getParam("/trajectoryPlanner/startEpsilon", startEpsilon)) {
		ROS_INFO("Node %s: retrieved parameter startEpsilon.", ros::this_node::getName().c_str());
	}
	else {
		ROS_FATAL("Node %s: unable to retrieve parameter startEpsilon.", ros::this_node::getName().c_str());
		return false;
	}
	
	if (Handle.getParam("/trajectoryPlanner/vehicleWidth", vehicleWidth)) {
		ROS_INFO("Node %s: retrieved parameter vehicleWidth.", ros::this_node::getName().c_str());
	}
	else {
		ROS_FATAL("Node %s: unable to retrieve parameter vehicleWidth.", ros::this_node::getName().c_str());
		return false;
	}
	
	if (Handle.getParam("/trajectoryPlanner/vehicleLength", vehicleLength)) {
		ROS_INFO("Node %s: retrieved parameter vehicleLength.", ros::this_node::getName().c_str());
	}
	else {
		ROS_FATAL("Node %s: unable to retrieve parameter vehicleLength.", ros::this_node::getName().c_str());
		return false;
	}
	
	if (Handle.getParam("/trajectoryPlanner/envType", envType)) {
		ROS_INFO("Node %s: retrieved parameter envType.", ros::this_node::getName().c_str());
	}
	else {
		ROS_FATAL("Node %s: unable to retrieve parameter envType.", ros::this_node::getName().c_str());
		return false;
	}
	
	if(Handle.getParam("/trajectoryPlanner/obsthresh", obsthresh)){
		ROS_INFO("Node %s: retrieved parameter obsthresh.", ros::this_node::getName().c_str());
	}
	else{
		ROS_FATAL("Node %s: unable to retrieve parameter obsthresh.", ros::this_node::getName().c_str());
		return false;
	}
	
	if(Handle.getParam("/trajectoryPlanner/cost_inscribed_thresh", cost_inscribed_thresh)){
		ROS_INFO("Node %s: retrieved parameter cost_inscribed_thresh.", ros::this_node::getName().c_str());
	}
	else{
		ROS_FATAL("Node %s: unable to retrieve parameter cost_inscribed_thresh.", ros::this_node::getName().c_str());
		return false;
	}
	
	if(Handle.getParam("/trajectoryPlanner/cost_possibly_circumscribed_thresh", cost_possibly_circumscribed_thresh)){
		ROS_INFO("Node %s: retrieved parameter cost_possibly_circumscribed_thresh.", ros::this_node::getName().c_str());
	}
	else{
		ROS_FATAL("Node %s: unable to retrieve parameter cost_possibly_circumscribed_thresh.", ros::this_node::getName().c_str());
		return false;
	}
	
	if(Handle.getParam("/trajectoryPlanner/cellsize_m", cellsize_m)){
		ROS_INFO("Node %s: retrieved parameter cellsize_m.", ros::this_node::getName().c_str());
	}
	else{
		ROS_FATAL("Node %s: unable to retrieve parameter cellsize_m.", ros::this_node::getName().c_str());
		return false;
	}
	
	if(Handle.getParam("/trajectoryPlanner/numTheta", numTheta)){
		ROS_INFO("Node %s: retrieved parameter numTheta.", ros::this_node::getName().c_str());
	}
	else{
		ROS_FATAL("Node %s: unable to retrieve parameter numTheta.", ros::this_node::getName().c_str());
		return false;
	}
	
	if(Handle.getParam("/trajectoryPlanner/allocated_time_secs_foreachplan", allocated_time_secs_foreachplan)){
		ROS_INFO("Node %s: retrieved parameter allocated_time_secs_foreachplan.", ros::this_node::getName().c_str());
	}
	else{
		ROS_FATAL("Node %s: unable to retrieve parameter allocated_time_secs_foreachplan.", ros::this_node::getName().c_str());
		return false;
	}
	
	if(Handle.getParam("/trajectoryPlanner/bsearchuntilfirstsolution", bsearchuntilfirstsolution)){
		ROS_INFO("Node %s: retrieved parameter bsearchuntilfirstsolution.", ros::this_node::getName().c_str());
	}
	else{
		ROS_FATAL("Node %s: unable to retrieve parameter bsearchuntilfirstsolution.", ros::this_node::getName().c_str());
		return false;
	}
	
	if(Handle.getParam("/trajectoryPlanner/bforwardsearch", bforwardsearch)){
		ROS_INFO("Node %s: retrieved parameter bforwardsearch.", ros::this_node::getName().c_str());
	}
	else{
		ROS_FATAL("Node %s: unable to retrieve parameter bforwardsearch.", ros::this_node::getName().c_str());
		return false;
	}
	
	if(envType == "xytheta"){
		if(Handle.getParam("/trajectoryPlanner/nominalvel", nominalvel)){
			ROS_INFO("Node %s: retrieved parameter nominalvel.", ros::this_node::getName().c_str());
		}
		else{
			ROS_FATAL("Node %s: unable to retrieve parameter nominalvel.", ros::this_node::getName().c_str());
			return false;
		}
		
		if(Handle.getParam("/trajectoryPlanner/timetoturn", timetoturn)){
			ROS_INFO("Node %s: retrieved parameter timetoturn.", ros::this_node::getName().c_str());
		}
		else{
			ROS_FATAL("Node %s: unable to retrieve parameter timetoturn.", ros::this_node::getName().c_str());
			return false;
		}
	}
	else{
		if(Handle.getParam("/trajectoryPlanner/numV", numV)){
			ROS_INFO("Node %s: retrieved parameter numV.", ros::this_node::getName().c_str());
		}
		else{
			ROS_FATAL("Node %s: unable to retrieve parameter numV.", ros::this_node::getName().c_str());
			return false;
		}
		
		if(Handle.getParam("/trajectoryPlanner/velocities", velocities)){
			ROS_INFO("Node %s: retrieved parameter velocities.", ros::this_node::getName().c_str());
		}
		else{
			ROS_FATAL("Node %s: unable to retrieve parameter velocities.", ros::this_node::getName().c_str());
			return false;
		}
		
		if(envType == "xythetavsteer"){
			if(Handle.getParam("/trajectoryPlanner/numSteers", numSteers)){
				ROS_INFO("Node %s: retrieved parameter numSteers.", ros::this_node::getName().c_str());
			}
			else{
				ROS_FATAL("Node %s: unable to retrieve parameter numSteers.", ros::this_node::getName().c_str());
				return false;
			}
		}
		else if(envType == "xythetavomega"){
			if(Handle.getParam("/trajectoryPlanner/numOmega", numOmega)){
				ROS_INFO("Node %s: retrieved parameter numOmega.", ros::this_node::getName().c_str());
			}
			else{
				ROS_FATAL("Node %s: unable to retrieve parameter numOmega.", ros::this_node::getName().c_str());
				return false;
			}
			
			if(Handle.getParam("/trajectoryPlanner/omegas", omegas)){
				ROS_INFO("Node %s: retrieved parameter omegas.", ros::this_node::getName().c_str());
			}
			else{
				ROS_FATAL("Node %s: unable to retrieve parameter omegas.", ros::this_node::getName().c_str());
				return false;
			}
		}
	}

	//init subscriber and publisher
	//SubscriberS = Handle.subscribe("roamfree/odometry", 10, &ROSnode::plannerGetStartCallback, this);
	SubscriberS = Handle.subscribe("RR/pose", 10, &ROSnode::plannerGetStartCallback, this);
	SubscriberG = Handle.subscribe("goal", 10, &ROSnode::plannerGetGoalCallback, this);
	SubscriberM = Handle.subscribe("map", 10, &ROSnode::plannerGetMapCallback, this);
	Publisher = Handle.advertise<quadrivio_msgs::PathWithVelocity>("path", 20);
	TestPublisher = Handle.advertise<nav_msgs::Path>("visualize_path", 20);

	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
	seq = 0;

	return true;
}

//void ROSnode::plannerGetStartCallback(const nav_msgs::Odometry::ConstPtr& start){
void ROSnode::plannerGetStartCallback(const geometry_msgs::PoseStamped::ConstPtr& start){
	tf::poseMsgToTF(start->pose, this->start);
	startv=0.1;
	startomega=0;
	haveStart = true;
	//ROS_INFO("Start received...");
}

void ROSnode::plannerGetGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
	tf::poseMsgToTF(goal->pose, this->goal);
	goalv = 0.1;
	goalomega = 0;
	haveGoal = true;
	ROS_INFO("Goal received...");
}

void ROSnode::plannerGetMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map){
	if(this->map!=NULL && (this->mapHeight!=map->info.height || this->mapWidth!=map->info.width)){
		delete[]this->map;
		this->map = NULL;
	}
	
	if(this->map == NULL){
		this->map = new double[map->info.height*map->info.width];
		this->mapHeight = map->info.height;
		this->mapWidth = map->info.width;
		this->changedCells.clear();
		
		for(int i=0;i<this->mapWidth*this->mapHeight;i++){
			this->map[i] = -1;
		}
	}
	
	int globalCnt = 0;
	
	for(int i=map->info.height-1;i>=0;i--){
		for(int j=0;j<map->info.width;j++){
			if(this->map[globalCnt]!=(double)(map->data[i*map->info.width + j])/100.0){
				sbpl_2Dcell_t tmpCell;
				
				tmpCell.x = globalCnt%this->mapWidth;
				tmpCell.y = this->mapHeight - 1 - globalCnt/this->mapHeight;
				changedCells.push_back(tmpCell);
				
				this->map[globalCnt]=(double)(map->data[i*map->info.width + j])/100.0;
			}
			
			globalCnt++;
		}
	}
	
	haveMap = true;
	
	ROS_INFO("Map received...");
}

int ROSnode::plan()
{
	if(!this->haveStart || !this->haveGoal || !this->haveMap)
		return -1;
	
	int bRet = 0;
	double dec_eps = startEpsilon/2;
	std::vector<double> oldStart;
	std::vector<double> oldGoal;
	MDPConfig MDPCfg;
	
	this->haveGoal = false;
	
	// set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
	std::vector<sbpl_2Dpt_t> perimeterptsV;
	sbpl_2Dpt_t pt_m;
	pt_m.x = -vehicleLength/2;
	pt_m.y = -vehicleWidth/2;
	perimeterptsV.push_back(pt_m);
	pt_m.x = vehicleLength/2;
	pt_m.y = -vehicleWidth/2;
	perimeterptsV.push_back(pt_m);
	pt_m.x = vehicleLength/2;
	pt_m.y = vehicleWidth/2;
	perimeterptsV.push_back(pt_m);
	pt_m.x = -vehicleLength/2;
	pt_m.y = vehicleWidth/2;
	perimeterptsV.push_back(pt_m);

	DiscreteSpaceInformation * environment;
	int initOk;

	oldStart.push_back(start.getOrigin().getX());
	oldStart.push_back(start.getOrigin().getY());
	oldStart.push_back(tf::getYaw(start.getRotation()));
	oldGoal.push_back(goal.getOrigin().getX());
	oldGoal.push_back(goal.getOrigin().getY());
	oldGoal.push_back(tf::getYaw(goal.getRotation()));
	
	if(envType == "xytheta"){
		environment = new EnvironmentNAVXYTHETALAT();
		
		((EnvironmentNAVXYTHETALAT *)environment)->SetEnvParameter("cost_inscribed_thresh", cost_inscribed_thresh);
		((EnvironmentNAVXYTHETALAT *)environment)->SetEnvParameter("cost_possibly_circumscribed_thresh", cost_possibly_circumscribed_thresh);
		
		initOk = ((EnvironmentNAVXYTHETALAT *)environment)->InitializeEnv(mapWidth, mapHeight, map, start.getOrigin().getX(), start.getOrigin().getY(), tf::getYaw(start.getRotation()), goal.getOrigin().getX(), goal.getOrigin().getY(), tf::getYaw(goal.getRotation()), 0.1, 0.1, 0.1, perimeterptsV, cellsize_m, nominalvel, timetoturn, obsthresh, primitivesFile.c_str());
		
		((EnvironmentNAVXYTHETALAT *)environment)->SetStart(start.getOrigin().getX(), start.getOrigin().getY(), tf::getYaw(start.getRotation()));
		((EnvironmentNAVXYTHETALAT *)environment)->SetGoal(goal.getOrigin().getX(), goal.getOrigin().getY(), tf::getYaw(goal.getRotation()));
	}
	else if(envType == "xythetav"){
		environment = new EnvironmentNAVXYTHETAV();
		
		initOk = ((EnvironmentNAVXYTHETAV *)environment)->InitializeEnv(mapWidth, mapHeight, numTheta, numV, velocities, map, start.getOrigin().getX(), start.getOrigin().getY(), tf::getYaw(start.getRotation()), startv, goal.getOrigin().getX(), goal.getOrigin().getY(), tf::getYaw(goal.getRotation()), goalv, perimeterptsV, cellsize_m, obsthresh, cost_inscribed_thresh, cost_possibly_circumscribed_thresh, primitivesFile.c_str());
		
		((EnvironmentNAVXYTHETAV *)environment)->SetStart(start.getOrigin().getX(), start.getOrigin().getY(), tf::getYaw(start.getRotation()), startv);
		((EnvironmentNAVXYTHETAV *)environment)->SetGoal(goal.getOrigin().getX(), goal.getOrigin().getY(), tf::getYaw(goal.getRotation()), goalv);
		
		oldStart.push_back(startv);
		oldGoal.push_back(goalv);
	}
	else if(envType == "xythetavsteer"){
		environment = new EnvironmentNAVXYTHETAVSTEER();
		
		initOk = ((EnvironmentNAVXYTHETAVSTEER *)environment)->InitializeEnv(mapWidth, mapHeight, numTheta, numV, numSteers, velocities, map, start.getOrigin().getX(), start.getOrigin().getY(), tf::getYaw(start.getRotation()), startv, 0, goal.getOrigin().getX(), goal.getOrigin().getY(), tf::getYaw(goal.getRotation()), goalv, 0, perimeterptsV, cellsize_m, obsthresh, cost_inscribed_thresh, cost_possibly_circumscribed_thresh, primitivesFile.c_str());
		
		((EnvironmentNAVXYTHETAVSTEER *)environment)->SetStart(start.getOrigin().getX(), start.getOrigin().getY(), tf::getYaw(start.getRotation()), startv, 0);
		((EnvironmentNAVXYTHETAVSTEER *)environment)->SetGoal(goal.getOrigin().getX(), goal.getOrigin().getY(), tf::getYaw(goal.getRotation()), goalv, 0);
		
		oldStart.push_back(startv);
		oldStart.push_back(0);
		oldGoal.push_back(goalv);
		oldGoal.push_back(0);
	}
	else if(envType == "xythetavomega"){
		environment = new EnvironmentNAVXYTHETAVOMEGA();
		
		initOk = ((EnvironmentNAVXYTHETAVOMEGA *)environment)->InitializeEnv(mapWidth, mapHeight, numTheta, numV, numOmega, velocities, omegas, map, start.getOrigin().getX(), start.getOrigin().getY(), tf::getYaw(start.getRotation()), startv, startomega, goal.getOrigin().getX(), goal.getOrigin().getY(), tf::getYaw(goal.getRotation()), goalv, goalomega, perimeterptsV, cellsize_m, obsthresh, cost_inscribed_thresh, cost_possibly_circumscribed_thresh, primitivesFile.c_str());
		
		((EnvironmentNAVXYTHETAVOMEGA *)environment)->SetStart(start.getOrigin().getX(), start.getOrigin().getY(), tf::getYaw(start.getRotation()), startv, startomega);
		((EnvironmentNAVXYTHETAVOMEGA *)environment)->SetGoal(goal.getOrigin().getX(), goal.getOrigin().getY(), tf::getYaw(goal.getRotation()), goalv, goalomega);
		
		oldStart.push_back(startv);
		oldStart.push_back(startomega);
		oldGoal.push_back(goalv);
		oldGoal.push_back(goalomega);
	}
	
	if(!initOk){
		ROS_FATAL("ERROR: Environment Initialization Failed...\n");
		return -2;
	}
	else{
		changedCells.clear();
	}

	// Initialize MDP Info
	if (!environment->InitializeMDPCfg(&MDPCfg)) {
		ROS_FATAL("ERROR: InitializeMDPCfg failed\n");
		return -3;
	}

	// plan a path
	std::vector<int> solution_stateIDs_V;

	SBPLPlanner* planner = new ADPlanner(environment, bforwardsearch);
	planner->set_initialsolution_eps(startEpsilon);
	((ADPlanner*)planner)->set_dec_eps(dec_eps);
	
	// set planner properties
	if (planner->set_start(MDPCfg.startstateid) == 0) {
		ROS_FATAL("ERROR: failed to set start state in planner\n");
		return -4;
	}
	if (planner->set_goal(MDPCfg.goalstateid) == 0) {
		ROS_FATAL("ERROR: failed to set goal state in planner\n");
		return -5;
	}
	
	planner->set_search_mode(bsearchuntilfirstsolution);
	
	int sc = 0;
	bool ne = false;
	bool flag = true;
	while(planner->get_solution_eps() > 1 && !ne){
		ROS_INFO("Start planning...");
		bRet = ((ADPlanner*)planner)->single_plan(allocated_time_secs_foreachplan, &solution_stateIDs_V, &sc);
		
		if(bRet){
			// publish the solution
			quadrivio_msgs::PathWithVelocity msg;
			nav_msgs::Path testMsg;
			
			
			if(envType == "xytheta"){
				std::vector<sbpl_xy_theta_pt_t> xythetaPath;
				((EnvironmentNAVXYTHETALAT *)environment)->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetaPath);
				
				msg.header.seq = seq++;
				msg.header.stamp = ros::Time::now();
				msg.header.frame_id = "/world";
				
				testMsg.header.seq = seq-1;
				testMsg.header.stamp = ros::Time::now();
				testMsg.header.frame_id = "/world";
				
				for(sbpl_xy_theta_pt_t pt:xythetaPath){
					geometry_msgs::PoseStamped p;
					p.header.frame_id="";
					p.header.stamp=ros::Time::now();
					p.header.seq=0;
					p.pose.position.x = pt.x;
					p.pose.position.y = pt.y;
					p.pose.orientation = tf::createQuaternionMsgFromYaw(pt.theta);
					
					msg.path.poses.push_back(p);
					testMsg.poses.push_back(p);
				}
			}
			else if(envType == "xythetav"){
				std::vector<sbpl_xy_theta_v_pt_t> xythetavPath;
				((EnvironmentNAVXYTHETAV *)environment)->ConvertStateIDPathintoXYThetaVPath(&solution_stateIDs_V, &xythetavPath);
				
				msg.header.seq = seq++;
				msg.header.stamp = ros::Time::now();
				msg.header.frame_id = "/world";
				
				testMsg.header.seq = seq-1;
				testMsg.header.stamp = ros::Time::now();
				testMsg.header.frame_id = "/world";
				
				for(sbpl_xy_theta_v_pt_t pt:xythetavPath){
					geometry_msgs::PoseStamped p;
					p.header.frame_id="";
					p.header.stamp=ros::Time::now();
					p.header.seq=0;
					p.pose.position.x = pt.x;
					p.pose.position.y = pt.y;
					p.pose.orientation = tf::createQuaternionMsgFromYaw(pt.theta);
					
					msg.path.poses.push_back(p);
					msg.velocities.push_back(pt.v);
					testMsg.poses.push_back(p);
				}
			}
			else if(envType == "xythetavsteer"){
				//Steer is not returned actually
				std::vector<sbpl_xy_theta_v_steer_pt_t> xythetavsteerPath;
				((EnvironmentNAVXYTHETAVSTEER *)environment)->ConvertStateIDPathintoXYThetaVSteerPath(&solution_stateIDs_V, &xythetavsteerPath);
				
				msg.header.seq = seq++;
				msg.header.stamp = ros::Time::now();
				msg.header.frame_id = "/world";
				
				testMsg.header.seq = seq-1;
				testMsg.header.stamp = ros::Time::now();
				testMsg.header.frame_id = "/world";
				
				for(sbpl_xy_theta_v_steer_pt_t pt:xythetavsteerPath){
					geometry_msgs::PoseStamped p;
					p.header.frame_id="";
					p.header.stamp=ros::Time::now();
					p.header.seq=0;
					p.pose.position.x = pt.x;
					p.pose.position.y = pt.y;
					p.pose.orientation = tf::createQuaternionMsgFromYaw(pt.theta);
					
					msg.path.poses.push_back(p);
					msg.velocities.push_back(pt.v);
					testMsg.poses.push_back(p);
				}
			}
			else if(envType == "xythetavomega"){
				ROS_FATAL("ERROR: Not yet implemented");
				return -6;
			}
			
			Publisher.publish(msg);
			TestPublisher.publish(testMsg);
			
			((ADPlanner*)planner)->set_dec_eps(planner->get_solution_eps()/2);
			ROS_INFO("End planning with %f epsilon: solution found", planner->get_solution_eps());
			
			if(changedCells.size()>0 && changedCells.size()<(mapHeight*mapWidth)*0.05){
				std::vector<int> changedCellsID;
				for(sbpl_2Dcell_t tmpCell:changedCells){
					int indexChanged = (mapHeight-1-tmpCell.y)*mapWidth + tmpCell.x;
					if(envType == "xytheta"){
						((EnvironmentNAVXYTHETALAT *)environment)->UpdateCost(tmpCell.x, tmpCell.y, map[indexChanged]);
					}
					else if(envType == "xythetav"){
						((EnvironmentNAVXYTHETAV *)environment)->UpdateCost(tmpCell.x, tmpCell.y, map[indexChanged]);
					}
					else if(envType == "xythetavsteer"){
						((EnvironmentNAVXYTHETAVSTEER *)environment)->UpdateCost(tmpCell.x, tmpCell.y, map[indexChanged]);
					}
					else if(envType == "xythetavomega"){
						((EnvironmentNAVXYTHETAVOMEGA *)environment)->UpdateCost(tmpCell.x, tmpCell.y, map[indexChanged]);
					}
				}
				
				changedCellsID.clear();
				
				if(envType == "xytheta"){
					((EnvironmentNAVXYTHETALAT *)environment)->GetPredsofChangedEdges(&changedCells, &changedCellsID);
				}
				else if(envType == "xythetav"){
					((EnvironmentNAVXYTHETAV *)environment)->GetPredsOfChangedEdges(&changedCells, &changedCellsID);
				}
				else if(envType == "xythetavsteer"){
					((EnvironmentNAVXYTHETAVSTEER *)environment)->GetPredsOfChangedEdges(&changedCells, &changedCellsID);
				}
				else if(envType == "xythetavomega"){
					((EnvironmentNAVXYTHETAVOMEGA *)environment)->GetPredsOfChangedEdges(&changedCells, &changedCellsID);
				}
				
				((ADPlanner *)planner)->update_preds_of_changededges(&changedCellsID);
				ROS_INFO("%d states were affected...\n", (int)changedCellsID.size());
				changedCells.clear();
			}
			else if(changedCells.size()>=(mapHeight*mapWidth)*0.05){
				ne = true;
				this->haveGoal = true;
				changedCells.clear();
				ROS_INFO("Too many changes in map... Restart planning!");
			}
			
			if(oldStart.at(0)!=start.getOrigin().getX() || oldStart.at(1)!=start.getOrigin().getY() || oldStart.at(2)!=tf::getYaw(start.getRotation()) || (envType == "xythetav" && (oldStart.at(3)!=startv)) || (envType == "xythetavsteer" && (oldStart.at(3)!=startv || oldStart.at(4)!=0)) || (envType == "xythetavomega" && (oldStart.at(3)!=startv || oldStart.at(4)!=startomega))){
				oldStart.clear();
				oldStart.push_back(start.getOrigin().getX());
				oldStart.push_back(start.getOrigin().getY());
				oldStart.push_back(tf::getYaw(start.getRotation()));
				
				planner->set_initialsolution_eps(planner->get_solution_eps()-dec_eps);
				int id;
				
				if(envType == "xytheta"){
					id = ((EnvironmentNAVXYTHETALAT *)environment)->SetStart(start.getOrigin().getX(), start.getOrigin().getY(), tf::getYaw(start.getRotation()));
				}
				else if(envType == "xythetav"){
					oldStart.push_back(startv);
					id = ((EnvironmentNAVXYTHETAV *)environment)->SetStart(start.getOrigin().getX(), start.getOrigin().getY(), tf::getYaw(start.getRotation()), startv);
				}
				else if(envType == "xythetavsteer"){
					oldStart.push_back(startv);
					oldStart.push_back(0);
					id = ((EnvironmentNAVXYTHETAVSTEER *)environment)->SetStart(start.getOrigin().getX(), start.getOrigin().getY(), tf::getYaw(start.getRotation()), startv, 0);
				}
				else if(envType == "xythetavomega"){
					oldStart.push_back(startv);
					oldStart.push_back(startomega);
					id = ((EnvironmentNAVXYTHETAVOMEGA *)environment)->SetStart(start.getOrigin().getX(), start.getOrigin().getY(), tf::getYaw(start.getRotation()), startv, startomega);
				}
				
				planner->set_start(id);
			}
			
			if(oldGoal.at(0)!=goal.getOrigin().getX() || oldGoal.at(1)!=goal.getOrigin().getY() || oldGoal.at(2)!=tf::getYaw(goal.getRotation()) || (envType == "xythetav" && (oldGoal.at(3)!=goalv)) || (envType == "xythetavsteer" && (oldGoal.at(3)!=goalv || oldGoal.at(4)!=0)) || (envType == "xythetavomega" && (oldGoal.at(3)!=goalv || oldStart.at(4)!=goalomega))){
				oldGoal.clear();
				oldGoal.push_back(goal.getOrigin().getX());
				oldGoal.push_back(goal.getOrigin().getY());
				oldGoal.push_back(tf::getYaw(goal.getRotation()));
				
				planner->set_initialsolution_eps(startEpsilon);
				((ADPlanner*)planner)->set_dec_eps(startEpsilon/2);
				int id;
				
				if(envType == "xytheta"){
					id = ((EnvironmentNAVXYTHETALAT *)environment)->SetGoal(goal.getOrigin().getX(), goal.getOrigin().getY(), tf::getYaw(goal.getRotation()));
				}
				else if(envType == "xythetav"){
					oldGoal.push_back(goalv);
					id = ((EnvironmentNAVXYTHETAV *)environment)->SetGoal(goal.getOrigin().getX(), goal.getOrigin().getY(), tf::getYaw(goal.getRotation()), goalv);
				}
				else if(envType == "xythetavsteer"){
					oldGoal.push_back(goalv);
					oldGoal.push_back(0);
					id = ((EnvironmentNAVXYTHETAVSTEER *)environment)->SetGoal(goal.getOrigin().getX(), goal.getOrigin().getY(), tf::getYaw(goal.getRotation()), goalv, 0);
				}
				else if(envType == "xythetavomega"){
					oldGoal.push_back(goalv);
					oldGoal.push_back(goalomega);
					id = ((EnvironmentNAVXYTHETAVOMEGA *)environment)->SetGoal(goal.getOrigin().getX(), goal.getOrigin().getY(), tf::getYaw(goal.getRotation()), goalv, goalomega);
				}
				
				planner->set_goal(id);
			}
		}
		else{
			ne = true;
			ROS_INFO("End planning with %f epsilon: solution not found", planner->get_solution_eps());
		}
	}

	delete environment;
	delete planner;

	ROS_INFO("End planning");
	
	return bRet;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, NAME_OF_THIS_NODE);

	ROSnode node;
	ros::Rate loopRate(15);

	if(!node.Prepare())
		return 1;
	
	while(ros::ok()){
		int val = 0;
		
		try{
			val=node.plan();
		}
		catch(std::exception * ex){
			ROS_INFO("Exception catched: %s...", ex->what());
		}
		
		if(val != 1){
			//ROS_INFO("ERROR: Plan failed");
		}
		else{
			ROS_INFO("SUCCESS: Planning executed");
		}
		
		ros::spinOnce();
		loopRate.sleep();
	}

	return (0);
}
