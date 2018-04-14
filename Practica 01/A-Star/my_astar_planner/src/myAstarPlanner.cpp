#include "../include/my_astar_planner/myAstarPlanner.h"
#include <pluginlib/class_list_macros.h>
#include <sstream>
#include <string>
#include <visualization_msgs/Marker.h>

PLUGINLIB_EXPORT_CLASS(myastar_planner::MyastarPlanner, nav_core::BaseGlobalPlanner)

namespace myastar_planner{
	multiset<coupleOfCells, FComparator>::iterator getPositionInList(multiset<coupleOfCells, FComparator> &list1, unsigned int cellID);
	
	bool isContains(multiset<coupleOfCells, FComparator> &list1, int cellID);
	
	MyastarPlanner::MyastarPlanner() : costmap_ros_(NULL), initialized_(false){
	
	}
	
	MyastarPlanner::MyastarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros) : costmap_ros_(NULL), initialized_(false){
		initialize(name, costmap_ros);
	}

  	void MyastarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros){
    		if(!initialized_){
    			porcentajePoda = 0.1;
    			pesoEstatico = 0.3;
      			costmap_ros_ = costmap_ros;
      			costmap_ = costmap_ros_->getCostmap();
      			ros::NodeHandle private_nh("~/" + name);
			private_nh.param("step_size", step_size_, costmap_->getResolution());
			private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
			plan_pub_ = private_nh.advertise<nav_msgs::Path>("planTotal", 1);
			marker_Open_publisher = private_nh.advertise<visualization_msgs::Marker>("open_list", 1000);
			marker_Closed_publisher = private_nh.advertise<visualization_msgs::Marker>("closed_list", 1000);
			marker_Goals_publisher = private_nh.advertise<visualization_msgs::Marker>("goals_markers", 1000);
			initialized_ = true;
		}else{
			ROS_WARN("This planner has already been initialized... doing nothing");
		}
  	}

	double MyastarPlanner::footprintCost(double x_i, double y_i, double theta_i){
		world_model_ = new base_local_planner::CostmapModel(*costmap_);
		if(!initialized_){
			ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
			return -1.0;
		}
		std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
		if(footprint.size() < 3){
			return -1.0;
		}
		double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
		return footprint_cost;
	}

	bool MyastarPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan){	
		if(!initialized_){
			ROS_ERROR("The astar planner has not been initialized, please call initialize() to use the planner");
			return false;
		}
		ROS_DEBUG("MyastarPlanner: Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
		plan.clear();
		closedList.clear();
		openList.clear();
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
		costmap_->worldToMap(goal_x, goal_y, mgoal_x, mgoal_y);
		cpgoal.index = MyastarPlanner::costmap_->getIndex(mgoal_x, mgoal_y);
		cpgoal.parent = 0;
		cpgoal.gCost = 0;
		cpgoal.hCost = 0;
		cpgoal.fCost = 0;
		double start_x = start.pose.position.x;
		double start_y = start.pose.position.y;
		unsigned int mstart_x, mstart_y;
		costmap_->worldToMap(start_x,start_y, mstart_x, mstart_y);
		cpstart.index = MyastarPlanner::costmap_->getIndex(mstart_x, mstart_y);
		cpstart.parent = cpstart.index;
		cpstart.gCost = 0;
		cpstart.hCost = MyastarPlanner::calculateHCost(cpstart.index,cpgoal.index);
		MyastarPlanner::openList.insert(cpstart);
		ROS_INFO("Inserto en Abiertos: %d", cpstart.index );
		ROS_INFO("Index del goal: %d", cpgoal.index );
		inicializaMarkersPoints(markers_OpenList, "openList", 0, 0.0f, 1.0f, 0.0f);
		inicializaMarkersPoints(markers_ClosedList, "closedList", 1, 1.0f, 0.0f, 0.0f);
		inicializaMarkersLine_List(markers_Goals, "goals", 2, 0.0f, 0.0f, 1.0f);
		limpiaMarkers(marker_Open_publisher, markers_ClosedList);
		limpiaMarkers(marker_Closed_publisher, markers_OpenList);
		visualizaCelda(marker_Open_publisher, markers_OpenList, cpstart.index);
		unsigned int explorados = 0;
		unsigned int currentIndex = cpstart.index;
		coupleOfCells COfCells;
		while (!MyastarPlanner::openList.empty()){
			COfCells = *(openList.begin());
			currentIndex = COfCells.index;
			openList.erase(openList.begin());
			closedList.insert(COfCells);
			visualizaCelda(marker_Closed_publisher, markers_ClosedList, COfCells.index);
			if(currentIndex == cpgoal.index){
				ROS_INFO("Se han explorado %u nodos y cerrados tiene %u nodos", explorados, (unsigned int)closedList.size());
				//ros::Duration(10).sleep();
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
				coupleOfCells currentCouple = COfCells; //ponia cpgoal
				unsigned int currentParent = COfCells.parent;
				ROS_INFO("Inserta en Plan GOAL: %f, %f PADRE: %u", pose.pose.position.x, pose.pose.position.y, currentParent);
				//ros::Duration(1).sleep();
				double secs1 = ros::Time::now().toSec();
				while (currentParent != cpstart.index){
					multiset<coupleOfCells, FComparator>::iterator it = getPositionInList(closedList, currentParent);
					coupleOfCells currentCouple;
					currentCouple.index = currentParent;
					currentCouple.parent = (*it).parent;
					currentCouple.gCost = (*it).gCost;
					currentCouple.hCost = (*it).hCost;
					currentCouple.fCost = (*it).fCost;
					unsigned int mpose_x, mpose_y;
					double wpose_x, wpose_y;
					costmap_->indexToCells((*it).index, mpose_x, mpose_y);
					costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
				        ROS_INFO("Las coordenadas de El PADRE de %u son (%u, %u) -> (%f, %f). Y su PADRE es %u.", currentParent, mpose_x,mpose_y,wpose_x, wpose_y, (*it).parent);
					//ros::Duration(1).sleep();
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
		        		ROS_INFO("Inserta en Plan: %f, %f", pose.pose.position.x, pose.pose.position.y);
		        		currentParent = (*it).parent;
				}
				double secs2 = ros::Time::now().toSec();
				float elapsed = secs2 - secs1;
				ROS_INFO("Tiempo Empleado: %f", elapsed);
				int size = openList.size();
				ROS_INFO("Cantidad Final de Nodos en Abiertos: %d", size);
				size = closedList.size();
				ROS_INFO("Cantidad Final de Nodos en Cerrados: %d", size);
				ROS_INFO("Distancia en Nodos: %ld", plan.size());
				geometry_msgs::PoseStamped inicio = plan[0], fin = plan[plan.size()-1];
				double fin_x = fin.pose.position.x;
				double fin_y = fin.pose.position.y;
				unsigned int mfin_x, mfin_y;
				costmap_->worldToMap(fin_x, fin_y, mfin_x, mfin_y);
				unsigned int indexFin = MyastarPlanner::costmap_->getIndex(mfin_x, mfin_y);
				double inicio_x = inicio.pose.position.x;
				double inicio_y = inicio.pose.position.y;
				unsigned int minicio_x, minicio_y;
				costmap_->worldToMap(inicio_x, inicio_y, minicio_x, minicio_y);
				unsigned int indexInicio = MyastarPlanner::costmap_->getIndex(minicio_x, minicio_y);
				double distancia = MyastarPlanner::calculateHCost(indexInicio, indexFin);
				ROS_INFO("Distancia en Metros: %f", distancia);
				ROS_INFO("Sale del bucle de generación del plan.");
				std::reverse(plan.begin(), plan.end());
				return true;
			}
			vector <unsigned int> neighborCells = findFreeNeighborCell(currentIndex);
			ROS_INFO("Ha encontrado %u vecinos", (unsigned int)neighborCells.size());
			vector <unsigned int> neighborNotInClosedList;
			for(uint i = 0; i < neighborCells.size(); ++i){
				if(!isContains(closedList, neighborCells[i])){
					neighborNotInClosedList.push_back(neighborCells[i]);
				}
			}
			ROS_INFO("Ha encontrado %u vecinos que no están en cerrados", (unsigned int)neighborNotInClosedList.size());
			vector <unsigned int> neighborsInOpenList;
			vector <unsigned int> neighborsNotInOpenList;
			for(uint i = 0; i < neighborNotInClosedList.size(); ++i){
				if(isContains(openList, neighborNotInClosedList[i])){
					neighborsInOpenList.push_back(neighborNotInClosedList[i]);
				}else{
					neighborsNotInOpenList.push_back(neighborNotInClosedList[i]);
				}
			}
			addNeighborCellsToOpenList(openList, neighborsNotInOpenList, currentIndex, COfCells.gCost, cpgoal.index);
			explorados++;
			visualizaLista(marker_Open_publisher, markers_OpenList, neighborsNotInOpenList);
			visualizaCelda(marker_Closed_publisher, markers_ClosedList, COfCells.index);
			actualizarCostes(openList, closedList, neighborNotInClosedList, COfCells.index, COfCells.gCost);
		}
		if(openList.empty()){
			ROS_INFO("Failure to find a path !");
			return false;
		}
	};
	
	double MyastarPlanner::calculateHCost(unsigned int start, unsigned int goal){
		unsigned int mstart_x, mstart_y, mgoal_x, mgoal_y;
		double wstart_x, wstart_y, wgoal_x, wgoal_y;
		costmap_->indexToCells(start, mstart_x, mstart_y);
		costmap_->mapToWorld(mstart_x, mstart_y, wstart_x, wstart_y);
		costmap_->indexToCells(goal, mgoal_x, mgoal_y);
		costmap_->mapToWorld(mgoal_x, mgoal_y, wgoal_x, wgoal_y);
		return sqrt((pow(wstart_x - wgoal_x,2))+pow(wstart_y - wgoal_y, 2));
	}

	bool MyastarPlanner::compareFCost(coupleOfCells const &c1, coupleOfCells const &c2){
		return c1.fCost < c2.fCost;
	}

	multiset<coupleOfCells, FComparator>::iterator getPositionInList(multiset<coupleOfCells, FComparator> &list1, unsigned int cellID){
		for(multiset<coupleOfCells, FComparator>::iterator it = list1.begin(); it != list1.end(); ++it){
			if (it->index == cellID){
				return it;
			}
   		}
	}

	vector <unsigned int> MyastarPlanner::findFreeNeighborCell(unsigned int CellID){
	        unsigned int mx, my;
        	double  wx, wy;
        	costmap_->indexToCells(CellID,mx,my);
        	ROS_INFO("Viendo vecinos de index: %u, Map coords: (%u,%u)", CellID, mx,my);
		vector <unsigned int>  freeNeighborCells;
		for(int x =-1; x <= 1; ++x){
			for(int y = -1; y <= 1; ++y){
				//ROS_INFO("A ver: X = %u, Size_X = %u, Y = %u Size_Y = %u",mx+x, (unsigned int)costmap_->getSizeInCellsX(),my+y, (unsigned int)costmap_->getSizeInCellsY());
				if((mx+x >= 0) && (mx+x < costmap_->getSizeInCellsX()) && (my+y >=0 ) && (my+y < costmap_->getSizeInCellsY())){
					costmap_->mapToWorld((unsigned int) mx+x, (unsigned int) my+y, wx, wy);
					//ROS_INFO("Comprobando casilla con Map coords(%u,%u), World coords (%f,%f)", mx+x, my+y ,wx,wy);
					if(costmap_->getCost(mx+x, my+y) < 127 && (!(x == 0 && y == 0))){
						unsigned int index = costmap_->getIndex(mx+x, my+y);
						//ROS_INFO("Vecina (%f, %f)", wx,wy);
						freeNeighborCells.push_back(index);
					}
				}
			}

		}
		return  freeNeighborCells;
	}

	bool isContains(multiset<coupleOfCells, FComparator> &list1, int cellID){
		for(multiset<coupleOfCells, FComparator>::iterator it = list1.begin(); it != list1.end(); ++it){
			if(it->index == cellID){
				return true;
			}
		}
		return false;
	}

	double MyastarPlanner::getMoveCost(unsigned int here, unsigned int there){
		return calculateHCost(here, there);
	}

	void MyastarPlanner::addNeighborCellsToOpenList(multiset<coupleOfCells, FComparator> &OPL, vector <unsigned int> neighborCells, unsigned int parent, float gCostParent, unsigned int goalCell){
		for(uint i = 0; i < neighborCells.size(); ++i){
			unsigned int x, y;
			costmap_->indexToCells(neighborCells[i], x, y);
			coupleOfCells CP;
			CP.index = neighborCells[i];
			CP.parent = parent;
			CP.gCost = gCostParent + getMoveCost(parent, neighborCells[i]) + footprintCost(x, y, 1);
			CP.hCost = calculateHCost(neighborCells[i], goalCell);
			CP.fCost = CP.gCost + CP.hCost;
			OPL.insert(CP);
		}
	}

	void MyastarPlanner::inicializaMarkersPoints(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b){
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

	void MyastarPlanner::inicializaMarkersLine_List(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b){
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

	void MyastarPlanner::visualizaCoords(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y){
		geometry_msgs::Point p;
		p.x = x;
		p.y = y;
		p.z = 0;
		marker.points.push_back(p);
		where.publish(marker);
		//points.points.pop_back();
        }

	void MyastarPlanner::visualizaCoordsLineUp(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y, double z){
        	marker.pose.position.x = x;
        	marker.pose.position.y = y;
        	where.publish(marker);
        	//points.points.pop_back();
	}

	void MyastarPlanner::visualizaCelda(ros::Publisher where, visualization_msgs::Marker &marker, unsigned int index){
		unsigned int mpose_x, mpose_y;
		double wpose_x, wpose_y;
		costmap_->indexToCells(index, mpose_x, mpose_y);
		costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
		visualizaCoords(where, marker, wpose_x, wpose_y);
	}

	void MyastarPlanner::visualizaLista(ros::Publisher where, visualization_msgs::Marker &marker, vector<unsigned int> lista){
		for(vector<unsigned int>::iterator i = lista.begin(); i != lista.end(); ++i){
			unsigned int mpose_x, mpose_y;
			double wpose_x, wpose_y;
			costmap_->indexToCells(*i, mpose_x, mpose_y);
			costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
			geometry_msgs::Point p;
			p.x = wpose_x;
			p.y = wpose_y;
			p.z = 0;
			marker.points.push_back(p);
		}
		where.publish(marker);
	}
	
	void MyastarPlanner::limpiaMarkers(ros::Publisher where, visualization_msgs::Marker &marker){
		if(!marker.points.empty()){
			marker.action = visualization_msgs::Marker::DELETE;
			where.publish(marker);
			marker.action = visualization_msgs::Marker::ADD;
		}
		marker.points.clear();
	}
	
	void MyastarPlanner::actualizarCostes(multiset<coupleOfCells, FComparator> &openList, multiset<coupleOfCells, FComparator> &closedList, const vector<unsigned int> &identificadores, const unsigned int &id_padre, const double &coste) {
		coupleOfCells aux;
		for (int i = 0; i < identificadores.size(); i++) {
			unsigned int id_actual = identificadores[i];
			set<coupleOfCells, FComparator>::iterator salida = openList.begin();
			for (set<coupleOfCells, FComparator>::iterator it = openList.begin(); it != openList.end(); it++) {
				if ((*it).index == id_actual)
					salida = it;
				else
					salida++;
			}
			if (salida != openList.end()) {
				unsigned int x, y;
				costmap_->indexToCells((*salida).index, x, y);
				double coste2 = coste + getMoveCost(id_padre, id_actual)  + footprintCost(x, y, 1);
				if (coste2 < (*salida).gCost) {
					aux.index = id_actual;
					aux.parent = id_padre;
					aux.gCost = coste2;
					aux.hCost = (*salida).hCost;
					aux.fCost = aux.gCost + aux.hCost;
					openList.erase(salida);
					openList.insert(aux);
				}
			}
		}
	}
	
}
