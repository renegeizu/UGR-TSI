#include "../include/my_astar_planner/myAstarPlanner.h"
#include <pluginlib/class_list_macros.h>
#include <sstream>
#include <string>
#include <visualization_msgs/Marker.h>

PLUGINLIB_EXPORT_CLASS(myastar_planner::MyastarPlanner, nav_core::BaseGlobalPlanner)

namespace myastar_planner {
	list<coupleOfCells>::iterator getPositionInList(list<coupleOfCells> & list1, unsigned int cellID);
	
	bool isContains(list<coupleOfCells> & list1, int cellID);
	
	MyastarPlanner::MyastarPlanner() : costmap_ros_(NULL), initialized_(false){
	
	}
	
	MyastarPlanner::MyastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL), initialized_(false){
		initialize(name, costmap_ros);
	}
	
	void MyastarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		if(!initialized_){
			costmap_ros_ = costmap_ros;
			costmap_ = costmap_ros_->getCostmap();
			ros::NodeHandle private_nh("~/" + name);
			private_nh.param("step_size", step_size_, costmap_->getResolution());
			private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
			plan_pub_ = private_nh.advertise<nav_msgs::Path>("planTotal",1);
			marker_Open_publisher = private_nh.advertise<visualization_msgs::Marker>("open_list", 1000);
			marker_Closed_publisher = private_nh.advertise<visualization_msgs::Marker>("closed_list", 1000);
			marker_Goals_publisher = private_nh.advertise<visualization_msgs::Marker>("goals_markers", 1000);
			costmapLimit = costmapLimitStart = 10;
			initialized_ = true;
		}else
			ROS_WARN("This planner has already been initialized... doing nothing");
	}

	bool MyastarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
		if(!initialized_){
			ROS_ERROR("The astar planner has not been initialized, please call initialize() to use the planner");
			return false;
		}
		ROS_DEBUG("MyastarPlanner: Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
		plan.clear();
		closedList.clear();
		abiertos.clear();
		costmap_ = costmap_ros_->getCostmap();
		if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
			ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
			return false;
		}
		tf::Stamped<tf::Pose> goal_tf;
		tf::Stamped<tf::Pose> start_tf;
		poseStampedMsgToTF(goal,goal_tf);
		poseStampedMsgToTF(start,start_tf);
		double useless_pitch, useless_roll, goal_yaw, start_yaw;
		start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
		goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);
		coupleOfCells cpstart, cpgoal;
		double goal_x = goal.pose.position.x;
		double goal_y = goal.pose.position.y;
		unsigned int mgoal_x, mgoal_y;
		costmap_->worldToMap(goal_x,goal_y,mgoal_x, mgoal_y);
		cpgoal.index = MyastarPlanner::costmap_->getIndex(mgoal_x, mgoal_y);
		cpgoal.parent = 0;
		cpgoal.gCost = 0;
		cpgoal.hCost = 0;
		cpgoal.fCost = 0;
		if (cpgoal.index != lastGoal.index)
			costmapLimit = costmapLimitStart;
		lastGoal = cpgoal;
		double start_x = start.pose.position.x;
		double start_y = start.pose.position.y;
		unsigned int mstart_x, mstart_y;
		costmap_->worldToMap(start_x,start_y, mstart_x, mstart_y);
		cpstart.index = MyastarPlanner::costmap_->getIndex(mstart_x, mstart_y);
		cpstart.parent =cpstart.index;
		cpstart.gCost = 0;
		cpstart.hCost = MyastarPlanner::calculateHCost(cpstart.index,cpgoal.index);
		cpstart.costmapValue = costmap_->getCost(mstart_x, mstart_y);
		abiertos.push(cpstart);
		ROS_INFO("Inserto en Abiertos: %d", cpstart.index );
		ROS_INFO("Index del goal: %d", cpgoal.index );
		inicializaMarkersPoints(markers_OpenList,"openList", 0,0.0f,1.0f,0.0f);
		inicializaMarkersPoints(markers_ClosedList,"closedList", 1,1.0f,0.0f,0.0f);
		inicializaMarkersLine_List(markers_Goals, "goals", 2, 0.0f, 0.0f,1.0f);
		limpiaMarkers(marker_Open_publisher, markers_ClosedList);
		limpiaMarkers(marker_Closed_publisher, markers_OpenList);
		visualizaCelda(marker_Open_publisher, markers_OpenList, cpstart.index);
		unsigned int explorados = 0;
		unsigned int currentIndex = cpstart.index;
		while (!abiertos.empty()) {
			coupleOfCells casillaActual = abiertos.pop();
			currentIndex = casillaActual.index;
			pair<unsigned, coupleOfCells> toInsert;
			toInsert.first = currentIndex;
			toInsert.second = casillaActual;
			closedList.insert(toInsert);
			visualizaCelda(marker_Closed_publisher, markers_ClosedList, currentIndex);
			if(currentIndex == cpgoal.index) {
				ROS_INFO("Se han explorado %u nodos y cerrados tiene %u nodos", explorados, (unsigned int)closedList.size());
				geometry_msgs::PoseStamped pose;
				pose.header.stamp =  ros::Time::now();
				pose.header.frame_id = goal.header.frame_id;
				pose.pose.position.x = goal_x;
				pose.pose.position.y = goal_y;
				pose.pose.position.z = 0.0;
				pose.pose.orientation.x = 0.0;
				pose.pose.orientation.y = 0.0;
				pose.pose.orientation.z = 0.0;
				pose.pose.orientation.w = 1.0;
				plan.push_back(pose);
				coupleOfCells currentCouple = cpgoal;
				unsigned int currentParent = casillaActual.index;
				ROS_INFO("Inserta en Plan GOAL: %f, %f PADRE: %u", pose.pose.position.x, pose.pose.position.y, currentParent);
				while (currentParent != cpstart.index) {
					map<unsigned, coupleOfCells>::iterator it=closedList.find(currentParent);
					coupleOfCells currentCouple;
					currentCouple.index=currentParent;
					currentCouple.parent=it->second.parent;
					currentCouple.gCost=it->second.gCost;
					currentCouple.hCost=it->second.hCost;
					currentCouple.fCost=it->second.fCost;
					unsigned int mpose_x, mpose_y;
					double wpose_x, wpose_y;
					costmap_->indexToCells(it->second.index, mpose_x, mpose_y);
					costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
					geometry_msgs::PoseStamped pose;
					pose.header.stamp =  ros::Time::now();
					pose.header.frame_id = goal.header.frame_id;
					pose.pose.position.x = wpose_x;
					pose.pose.position.y = wpose_y;
					pose.pose.position.z = 0.0;
					pose.pose.orientation.x = 0.0;
					pose.pose.orientation.y = 0.0;
					pose.pose.orientation.z = 0.0;
					pose.pose.orientation.w = 1.0;
					plan.push_back(pose);
					currentParent = it->second.parent;
				}
				unsigned int mpose_x, mpose_y;
				double wpose_x, wpose_y;
				costmap_->indexToCells(cpstart.index, mpose_x, mpose_y);
				costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
				pose.header.stamp =  ros::Time::now();
				pose.header.frame_id = goal.header.frame_id;
				pose.pose.position.x = wpose_x;
				pose.pose.position.y = wpose_y;
				pose.pose.position.z = 0.0;
				pose.pose.orientation.x = 0.0;
				pose.pose.orientation.y = 0.0;
				pose.pose.orientation.z = 0.0;
				pose.pose.orientation.w = 1.0;
				plan.push_back(pose);
				ROS_INFO("Sale del bucle de generación del plan.");
				std::reverse(plan.begin(),plan.end());
				publishPlan(plan);
				return true;
			}
			JPSobtainSuccessors(casillaActual.index != casillaActual.parent, casillaActual.parent, casillaActual.index, casillaActual.gCost, cpgoal.index);
			explorados++;
		}
		if(abiertos.empty()) {
			ROS_INFO("No ha sido posible trazar un camino!");
			if (costmapLimit + 10  <= 127)
				costmapLimit += 10;
			else {
				costmapLimit +=1;
			}
			ROS_INFO("[COSTMAP] Permitimos hasta limite %u", costmapLimit);
			return false;
		}
	}

	bool MyastarPlanner::isValidCell(unsigned cx, unsigned cy) {
		return cx >= 0 && cx < costmap_->getSizeInCellsX() && cy >= 0 && cy < costmap_->getSizeInCellsY();
	}

	vector<unsigned> MyastarPlanner::JPSfindNeighbours(bool hasParent, unsigned parent, unsigned currentNode) {
		vector<unsigned> neighborhood;
		if (hasParent) {
			unsigned cx, cy, px, py;
			costmap_->indexToCells(parent, px, py);
			costmap_->indexToCells(currentNode, cx, cy);
			int dx = (cx*1.0-px*1-0)/max(abs(cx*1.0-px*1.0), 1.0);
			int dy = (cy*1.0-py*1.0)/max(abs(cy*1.0-py*1.0), 1.0);
			if (dx != 0 && dy != 0) {
				bool moveX = false, moveY = false;
				if (isValidMoveCell(cx, cy+dy)) {
					unsigned index = costmap_->getIndex(cx, cy+dy);
					neighborhood.push_back(index);
					moveY = true;
				}
				if (isValidMoveCell(cx+dx, cy)) {
					unsigned index = costmap_->getIndex(cx+dx, cy);
					neighborhood.push_back(index);
					moveX = true;
				}
				if (moveX || moveY) {
					unsigned index = costmap_->getIndex(cx+dx, cy+dy);
					neighborhood.push_back(index);
				}
				if (!isValidMoveCell(cx-dx,cy) && moveY) {
					unsigned index = costmap_->getIndex(cx-dx, cy+dy);
					neighborhood.push_back(index);
				}
				if (!isValidMoveCell(cx,cy-dy) && moveX) {
					unsigned index = costmap_->getIndex(cx+dx, cy-dy);
					neighborhood.push_back(index);
				}
			}else if (dx == 0) {
				if (isValidMoveCell(cx, cy + dy)) {
					unsigned index = costmap_->getIndex(cx, cy+dy);
					neighborhood.push_back(index);
				}
				if (!isValidMoveCell(cx+1, cy)) {
					unsigned index = costmap_->getIndex(cx+1, cy+dy);
					neighborhood.push_back(index);
				}
				if (!isValidMoveCell(cx-1, cy)) {
					unsigned index = costmap_->getIndex(cx-1, cy+dy);
					neighborhood.push_back(index);
				}
			}else if (dy == 0) {
				if (isValidMoveCell(cx + dx, cy)) {
					unsigned index = costmap_->getIndex(cx+dx, cy);
					neighborhood.push_back(index);
				}
				if (!isValidMoveCell(cx, cy+1)) {
					unsigned index = costmap_->getIndex(cx+dx, cy+1);
					neighborhood.push_back(index);
				}
				if (!isValidMoveCell(cx, cy-1)) {
					unsigned index = costmap_->getIndex(cx+dx, cy-1);
					neighborhood.push_back(index);
				}
			}
		}else {
			return findFreeNeighborCell(currentNode);
		}
		return neighborhood;
	}

	pair<bool, unsigned> MyastarPlanner::JPSJump(bool hasNode, unsigned currentNode, unsigned parent, unsigned endNode) {
		pair<bool,unsigned> p;
		p.first = false;
		if (!hasNode)
			return p;
		unsigned cx, cy, px, py;
		costmap_->indexToCells(parent, px, py);
		costmap_->indexToCells(currentNode, cx, cy);
		int dx = cx-px;
		int dy = cy-py;
		if (isValidCell(cx,cy) && !isValidMoveCell(cx,cy))
			return p;
		if (currentNode == endNode) {
			p.first = true;
			p.second = currentNode;
			return p;
		}
		if (dx != 0 && dy != 0) {
			if ((isValidMoveCell(cx-dx, cy+dy) && isValidCell(cx-dx,cy) && !isValidMoveCell(cx-dx,cy)) || (isValidMoveCell(cx+dx, cy-dy) && isValidCell(cx,cy-dy) && !isValidMoveCell(cx,cy-dy))) {
				p.first = true;
				p.second = currentNode;
				return p;
			}
		}else if (dx != 0) {
			if ((isValidMoveCell(cx+dx, cy+1) && isValidCell(cx,cy+1) && !isValidMoveCell(cx,cy+1)) || (isValidMoveCell(cx+dx, cy-1) && isValidCell(cx,cy-1) && !isValidMoveCell(cx,cy-1))) {
				p.first = true;
				p.second = currentNode;
				return p;
			}
		}else {
			if ((isValidMoveCell(cx+1, cy+dy) && isValidCell(cx+1,cy) && !isValidMoveCell(cx+1,cy)) || (isValidMoveCell(cx-1, cy+dy) && isValidCell(cx-1,cy) && !isValidMoveCell(cx-1,cy))) {
				p.first = true;
				p.second = currentNode;
				return p;
			}
		}
		if (dx != 0 && dy != 0) {
			p.first = true;
			p.second = currentNode;
			pair<bool, unsigned> check;
			if (isValidCell(cx+dx,cy)) {
				unsigned newNode = costmap_->getIndex(cx+dx, cy);
				check = JPSJump(hasNode, newNode, currentNode, endNode);
				if (check.first)
					return p;
			}
			if (isValidCell(cx, cy+dy)) {
				unsigned newNode = costmap_->getIndex(cx, cy+dy);
				check = JPSJump(hasNode, newNode, currentNode, endNode);
				if (check.first)
					return p;
			}
		}
		if (isValidMoveCell(cx+dx,cy) || isValidMoveCell(cx, cy+dy)) {
			unsigned newNode = costmap_->getIndex(cx+dx, cy+dy);
			return JPSJump(hasNode, newNode, currentNode, endNode);
		}
		return p;
	}

	void MyastarPlanner::JPSobtainSuccessors(bool hasParent, unsigned parent, unsigned currentNode, float gCostParent, unsigned endNode) {
		vector<unsigned> neighborhood = JPSfindNeighbours(hasParent, parent, currentNode);
		for (unsigned i = 0; i < neighborhood.size(); ++i) {
			pair<bool,unsigned> myJump = JPSJump(true,neighborhood[i],currentNode, endNode);
			if (myJump.first) {
				unsigned jumpNode = myJump.second;
				if (closedList.find(jumpNode)==closedList.end()) {
					vector<unsigned> toAdd;
					toAdd.push_back(jumpNode);
					addNeighborCellsToOpenList(toAdd, currentNode, gCostParent, endNode);
				}
			}
		}
	}

	bool MyastarPlanner::checkConditions(double currentX, double currentY, double nx, double ny, const list<geometry_msgs::PoseStamped>::iterator &it) {
		bool checkX;
		if (nx > 0)
			checkX = currentX < it->pose.position.x;
		else if (nx == 0)
			checkX = true;
		else
			checkX = currentX > it->pose.position.x;
		bool checkY;
		if (ny > 0)
			checkY = currentY < it->pose.position.y;
		else if (ny == 0)
			checkY = true;
		else
			checkY = currentY > it->pose.position.y;
		return checkX && checkY;
	}

	double MyastarPlanner::calculateHCost(unsigned int start, unsigned int goal) {
		unsigned int mstart_x, mstart_y, mgoal_x, mgoal_y;
		double wstart_x, wstart_y, wgoal_x, wgoal_y;
		costmap_->indexToCells(start, mstart_x, mstart_y);
		costmap_->mapToWorld(mstart_x, mstart_y, wstart_x, wstart_y);
		costmap_->indexToCells(goal, mgoal_x, mgoal_y);
		costmap_->mapToWorld(mgoal_x, mgoal_y, wgoal_x, wgoal_y);
		return sqrt((pow(wstart_x - wgoal_x,2))+pow(wstart_y - wgoal_y, 2));
	}

	bool MyastarPlanner::compareFCost(coupleOfCells const &c1, coupleOfCells const &c2) {
		return c1.fCost < c2.fCost;
	}

	list<coupleOfCells>::iterator getPositionInList(list<coupleOfCells> & list1, unsigned int cellID) {
		for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++){
			if (it->index == cellID)
				return it;
		}
	}

	vector <unsigned int> MyastarPlanner::findFreeNeighborCell (unsigned int CellID){
		unsigned int mx, my;
		double  wx, wy;
		costmap_->indexToCells(CellID,mx,my);
		ROS_INFO("Viendo vecinos de index: %u, Map coords: (%u,%u)", CellID, mx,my);
		vector <unsigned int>  freeNeighborCells;
		for (int x=-1;x<=1;x++)
			for (int y=-1; y<=1;y++){
				if ((mx+x>=0)&&(mx+x < costmap_->getSizeInCellsX())&&(my+y >=0 )&&(my+y < costmap_->getSizeInCellsY())){
					costmap_->mapToWorld( (unsigned int) mx+x, (unsigned int) my+y, wx, wy);
					if(costmap_->getCost(mx+x,my+y) < costmapLimit  && (!(x==0 && y==0))){
						unsigned int index = costmap_->getIndex(mx+x,my+y);
						freeNeighborCells.push_back(index);
					}
				}
			}
		return  freeNeighborCells;
	}

	double MyastarPlanner::getMoveCost(unsigned int here, unsigned int there) {
		return calculateHCost(here,there);
	}

	void MyastarPlanner::addNeighborCellsToOpenList(vector <unsigned int>& neighborCells, unsigned int parent, float gCostParent, unsigned int goalCell) {
		vector <coupleOfCells> neighborsCellsOrdered;
		for(uint i = 0; i < neighborCells.size(); i++) {
			coupleOfCells CP;
			CP.index=neighborCells[i];
			CP.parent = parent;
			CP.gCost = gCostParent + getMoveCost(parent, CP.index);
			CP.hCost = getMoveCost(CP.index, goalCell);
			unsigned mx, my;
			costmap_->indexToCells(CP.index, mx, my);
			if (isValidCell(mx,my))
				CP.costmapValue = costmap_->getCost(mx,my);
			else
				CP.costmapValue = 500;
			CP.fCost = CP.gCost + CP.hCost;
			if (closedList.find(CP.index)==closedList.end()) {
				abiertos.push(CP);
				visualizaCelda(marker_Open_publisher, markers_OpenList, CP.index);
			}
		}
	}
      
	void MyastarPlanner::inicializaMarkersPoints(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b) {
        	marker.header.frame_id = costmap_ros_->getGlobalFrameID().c_str();
        	marker.header.stamp =  ros::Time::now();
        	marker.ns = ns;
	        marker.action = visualization_msgs::Marker::ADD;
        	marker.pose.orientation.w = 0.0;
	        marker.id = id;
		marker.type = visualization_msgs::Marker::POINTS;
		marker.scale.x = costmap_->getResolution();
        	marker.scale.y = costmap_->getResolution();
		marker.color.g = g;
        	marker.color.r = r;
        	marker.color.b = b;
        	marker.color.a = 1.0;
	}

	void MyastarPlanner::inicializaMarkersLine_List(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b) {
        	marker.header.frame_id = costmap_ros_->getGlobalFrameID().c_str();
        	marker.header.stamp =  ros::Time::now();
	        marker.ns = ns;
        	marker.action = visualization_msgs::Marker::ADD;
        	marker.pose.orientation.w = 0.0;
        	marker.pose.position.x = 0.0;
        	marker.pose.position.y = 0.0;
	        marker.id = id;
	        marker.type = visualization_msgs::Marker::SPHERE;
		marker.scale.x = marker.scale.y = 0.5;
        	marker.color.g = g;
        	marker.color.r = r;
        	marker.color.b = b;
		marker.color.a = 1.0;
	}

	void MyastarPlanner::visualizaCoords(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y) {
		geometry_msgs::Point p;
        	p.x = x;
        	p.y = y;
        	p.z = 0;
        	marker.points.push_back(p);
        	marker.action = visualization_msgs::Marker::ADD;
        	where.publish(marker);
        }

	void MyastarPlanner::visualizaCelda(ros::Publisher where, visualization_msgs::Marker &marker, unsigned int index) {
		unsigned int mpose_x, mpose_y;
		double wpose_x, wpose_y;
		costmap_->indexToCells(index, mpose_x, mpose_y);
		costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
		visualizaCoords(where, marker, wpose_x, wpose_y);
	}

	void MyastarPlanner::limpiaMarkers(ros::Publisher where, visualization_msgs::Marker &marker) {
		if (!marker.points.empty()){
			marker.action = visualization_msgs::Marker::DELETE;
			where.publish(marker);
			marker.action = visualization_msgs::Marker::ADD;
		}
		marker.points.clear();
	}

	bool operator<(const coupleOfCells &c1, const coupleOfCells &c2) {
		if (c1.fCost == c2.fCost)
			return c1.costmapValue < c2.costmapValue;
		return c1.fCost < c2.fCost;
	}

	void MyPriorityQueue::push(const coupleOfCells &c) {
		abiertosIndex.insert(c.index);
		abiertos.push(c);
	}

	coupleOfCells MyPriorityQueue::pop() {
		coupleOfCells mejor = abiertos.top();
		abiertosIndex.erase(mejor.index);
		abiertos.pop();
		return mejor;
	}

	void MyPriorityQueue::clear() {
		abiertosIndex.clear();
		abiertos = priority_queue<coupleOfCells, vector<coupleOfCells>, greater<coupleOfCells> >();
	}

	bool MyPriorityQueue::empty() {
		return abiertosIndex.empty();
	}

}
