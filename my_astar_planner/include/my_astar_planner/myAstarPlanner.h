/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  Copyright (c) 2015, Juan Fdez-Olivares
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Juan Fdez-Olivares, Eitan Marder-Eppstein
*********************************************************************/
#ifndef MYASTAR_PLANNER_H_
#define MYASTAR_PLANNER_H_

//includes para integrarse en ROS
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h> //??
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

//includes específicos para hacer referencia a la implementación del algoritmo astar.

#include <vector>
#include <list>
#include <queue>  // Priority_Queue
#include <functional>


namespace myastar_planner{
  /**
  * @class myastar_planner
  * @brief Provides an a-star simple global planner that will compute a valid goal point for the local planner by using classical astar implementation.
  */

  /**
  * @struct coupleOfCells
  * @brief A struct that represents a node, that is, a couple of current and parent cells
  */
  class coupleOfCells {
  public:
    unsigned int index;
    unsigned int parent;
    double gCost;
    double hCost;
    double fCost;
    bool operator>(const coupleOfCells& other) const {
      return fCost > other.fCost;
    }
  };

  class MyCostmapModel : public base_local_planner::CostmapModel {
  public:
    MyCostmapModel(costmap_2d::Costmap2D& c) :CostmapModel(c) {}
  };

  class MyastarPlanner : public nav_core::BaseGlobalPlanner { //implementa la interfaz que provee nav_core::BaseGlobalPlanner
  public:
    /**
    * @brief  Constructor for the MyastarPlanner
    */
    MyastarPlanner();
    /**
    * @brief  Constructor for the MyastarPlanner
    * @param  name The name of this planner
    * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
    */
    MyastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
    * @brief  Initialization function for MyastarPlanner
    * @param  name The name of this planner
    * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
    */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
    * @brief Given a goal pose in the world, compute a plan
    * @param start The start pose
    * @param goal The goal pose
    * @param plan The plan... filled by the planner
    * @return True if a valid plan was found, false otherwise
    */
    bool makePlan(const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

  private:

    //necesarios para manejar el costmap y el footprint.
    costmap_2d::Costmap2DROS* costmap_ros_;
    double step_size_, min_dist_from_robot_;
    costmap_2d::Costmap2D* costmap_;

	std::list<coupleOfCells> openList; //!< the open list: it contains all the expanded cells (current cells)
	std::list<coupleOfCells> closedList; //!< the closed list: contains the explored cells


    //para publicar el plan
    ros::Publisher plan_pub_;

    /**
    * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
    * @param x_i The x position of the robot
    * @param y_i The y position of the robot
    * @param theta_i The orientation of the robot
    * @return
    */
    double footprintCost(double x_i, double y_i, double theta_i);
    /**
    * @brief  Calcula la estimación del costo de ir desde una casilla (definida por su indice) hasta otra (definida por su índice)
    * @param start El índice de la casilla inicial
    * @param end El índice de la casilla final
    * @return
    */
    double calculateHCost(unsigned int start, unsigned int goal);

    // Devuelve true si f(c1) < f(c2)
    static bool compareFCost(coupleOfCells const &c1, coupleOfCells const &c2);

    // Devuelve celdas adyacentes a CellID que estén libres
    std::vector <unsigned int> findFreeNeighborCell (unsigned int CellID);

    /*******************************************************************************/
    //Function Name: addNeighborCellsToOpenList
    //Inputs: the open list, the neighbors Cells and the parent Cell
    //Output:
    //Description: it is used to add the neighbor Cells to the open list
    /*********************************************************************************/
      void addNeighborCellsToOpenList(std::list<coupleOfCells> & OPL, std::vector <unsigned int> neighborCells, unsigned int parent, float gCostParent, unsigned int goalCell, unsigned int start);

      double getMoveCost(unsigned int here, unsigned int there);

      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);


      bool initialized_;
  };
};
#endif
