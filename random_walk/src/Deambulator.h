#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Deambulator{
private:
  ros::NodeHandle node;
  ros::Publisher commandPub;
  ros::Subscriber laserSub;
  bool keepMoving;

  //Metodos
  void moveForward();
  void rotate();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

public:
  //Parametros modificables
  const static double FORWARD_SPEED_MPS = 0.2;
  const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
  const static double MAX_SCAN_ANGLE_RAD = -30.0/180*M_PI;

  double MIN_PROXIMITY_RANGE_M; //tiene que ser menor que el rango maximo del laser, no se establece a static si no falla
  double ROTATION_VEL;

  Deambulator();
  Deambulator(double prox);
  void startMoving();
  void setProximityRange(double prox);
  void setRotationVelocity(double vel);
};
