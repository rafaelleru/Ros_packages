#include "Deambulator.h"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "deambulador");

  Deambulator deambulador;
  deambulador.setProximityRange(strtod (argv[1], NULL));
  deambulador.setRotationVelocity(strtod (argv[2], NULL));
  deambulador.startMoving();
  return 0;
}
