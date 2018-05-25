#ifndef Project3_hpp
#define Project3_hpp

#include <stdio.h>
#include <vector>
#include "Robot.h"
#include "Vector2D.h"
#include "Simulator.h"

class Project3 {
private:
  float utilities[10][40];
  float epsilon;
  RobotAction directions[10][40];
  char arrow[10][40];
  float pc;
  float reward;
  float discountValue;
public:
    /**
     * @brief default constructor
     */
    Project3(Simulator* sim1);

    /**
     * @brief get optimal action
     * @param sim1 simulator pointer
     * @param r robot pointer
     * @return optimal action
     */
    RobotAction getOptimalAction(Simulator* sim1, Robot* r1);
    void calculateUtilities(Simulator* sim1);
    float getMax(float left, float right, float up, float down);
    void getDirections();
};

#endif
