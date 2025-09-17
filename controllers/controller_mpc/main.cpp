#include <webots/Robot.hpp>

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  while (robot->step(timeStep) != -1) {
  };

  delete robot;
  return 0;
}
