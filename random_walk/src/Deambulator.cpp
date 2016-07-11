#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "Deambulator.h"
#include <time.h>       /* time */

Deambulator::Deambulator(){
  MIN_PROXIMITY_RANGE_M = 0.5; //Menor que el rango maximo del sensor
  ROTATION_VEL = 1.0;
  keepMoving = true;

  //notificar nuevo topic para la velicidad del robot
  commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  //se suscribe al topic del LaserScan
  laserSub = node.subscribe("base_scan", 1, &Deambulator::scanCallback, this);

  //inicializar el Ramndon
  srand (time(NULL));
}

Deambulator::Deambulator(double prox){
  MIN_PROXIMITY_RANGE_M = prox;
  Deambulator();
}

void Deambulator::moveForward(){
  geometry_msgs::Twist msg; //mensaje
  msg.linear.x = FORWARD_SPEED_MPS;
  commandPub.publish(msg);
}

void Deambulator::rotate(){
  geometry_msgs::Twist rotation; //mensaje
  rotation.angular.z = ROTATION_VEL;
  commandPub.publish(rotation);
}

void Deambulator::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
  //establecemos los indices para la exploracion del vector que devuelve el laser
  //en funcion de los angulos de escaneo establecidos
  int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min)/scan->angle_increment);
  int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - scan->angle_min)/scan->angle_increment);

  float closestRange = scan->ranges[minIndex]; //vector con cosas del laser
  for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
    if(scan->ranges[currIndex] < closestRange)
      closestRange = scan->ranges[currIndex];
  }

  ROS_INFO_STREAM("closestRange: " << closestRange);

  if (closestRange < MIN_PROXIMITY_RANGE_M) {
    ROS_INFO("Stop");
    keepMoving = false;
  } else{
    ROS_INFO("Nos vamos");
    keepMoving = true;
  }
}

void Deambulator::startMoving(){
  ros::Rate rate(10); //frecuencia en hz

  while (ros::ok()) {
    if (!keepMoving){
      int new_rate = rand() % 60 + 1;
      for (size_t i = 0; i < new_rate; i++) {
        rotate();
      }
    }else
      moveForward();

    ros::spinOnce();
    rate.sleep();
  }
}

void Deambulator::setProximityRange(double prox){
  MIN_PROXIMITY_RANGE_M = prox;
}

void Deambulator::setRotationVelocity(double vel){
  ROTATION_VEL = vel;
}
