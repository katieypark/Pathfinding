#include "Project3.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

/**
* @brief default constructor
*/
Project3::Project3(Simulator* sim1) {
	// Here, you should initialize the grid with all the known obstacles.
  epsilon = 0.001;
  pc = 0.5;
  reward = -0.04;
  discountValue = 0.9f;

  //Initialize utility values
  for (int i=0; i<sim1->getHeight(); i++) {
    for (int j=0; j<sim1->getWidth(); j++) {
      utilities[i][j] = 0.0;
      directions[i][j] = (RobotAction)MOVE_RIGHT;
      arrow[i][j] = '^';
    }
  }

  //Set utility values for all obstacles
  std::vector<Point2D> obstacles = sim1->getKnownObstacleLocations();
  for (int i=0; i<obstacles.size(); i++) {
    Point2D current = obstacles[i];
    utilities[(int)current.x][(int)current.y] = -1.0;
  }

  //Set utility for target
  Point2D target = sim1->getTarget();
  utilities[(int)target.x][(int)target.y] = 1.0;

  calculateUtilities(sim1);
  getDirections();

  // for (int i=0; i<sim1->getHeight(); i++) {
  //   for (int j=0; j<sim1->getWidth(); j++) {
  //     std::cout << utilities[i][j] << std::endl;
  //   }
  // }
}

void Project3::getDirections() {
  float up;
  float left;
  float right;
  float down;
  float maxval;
  for (int i=0; i<10; i++) {
    for (int j=0; j<40; j++) {
      up = -100.0;
      left = -100.0;
      right = -100.0;
      down = -100.0;
      if (i > 0) {
        up = utilities[i-1][j];
      }
      if (j > 0) {
        left = utilities[i][j-1];
      }
      if (i < 9) {
        down = utilities[i+1][j];
      }
      if (j < 39) {
        right = utilities[i][j+1];
      }
      maxval = getMax(left, right, up, down);
      if (maxval == up) {
        directions[i][j] = (RobotAction)MOVE_UP;
        arrow[i][j] = '^';
      } else if (maxval == left) {
        directions[i][j] = (RobotAction)MOVE_LEFT;
        arrow[i][j] = '<';
      } else if (maxval == right) {
        directions[i][j] = (RobotAction)MOVE_RIGHT;
        arrow[i][j] = '>';
      } else if (maxval == down) {
        directions[i][j] = (RobotAction)MOVE_DOWN;
        arrow[i][j] = 'v';
      }
    }
    std::cout << std::endl;
  }
}

void Project3::calculateUtilities(Simulator* sim1) {
  float maxUtilityChange;
  float utilitiesprime[10][40];
  float left;
  float right;
  float down;
  float up;
  float maxval;

  //value iteration
  do {
    maxUtilityChange = 0.0;
    left = 0;
    right = 0;
    down = 0;
    up = 0;
    maxval = 0;
    for (int i=0; i<sim1->getHeight(); i++) {
      for (int j=0; j<sim1->getWidth(); j++) {
        utilitiesprime[i][j] = utilities[i][j];
      }
    }
    //calculate utilities
    for (int i=0; i<sim1->getHeight(); i++) {
      for (int j=0; j<sim1->getWidth(); j++) {
        //if goal or obstacle
        if (utilitiesprime[i][j] == -1.0 || utilitiesprime[i][j] == 1.0) {
          continue;
        }
        else {
          if ((i > 0) && (i < 9) && (j > 0) && (j < 39)) {
            up = (pc * utilitiesprime[i-1][j]) + (((1-pc)/3) * utilitiesprime[i+1][j]) + (((1-pc)/3) * utilitiesprime[i][j+1])
                    + (((1-pc)/3) * utilitiesprime[i][j-1]);
            left = (pc * utilitiesprime[i][j-1]) + (((1-pc)/3) * utilitiesprime[i-1][j]) + (((1-pc)/3) * utilitiesprime[i+1][j])
                    + (((1-pc)/3) * utilitiesprime[i][j+1]);
            down = (pc * utilitiesprime[i+1][j]) + (((1-pc)/3) * utilitiesprime[i-1][j]) + (((1-pc)/3) * utilitiesprime[i][j-1])
                    + (((1-pc)/3) * utilitiesprime[i][j+1]);
            right = (pc * utilitiesprime[i][j+1]) + (((1-pc)/3) * utilitiesprime[i-1][j]) + (((1-pc)/3) * utilitiesprime[i][j-1])
                    + (((1-pc)/3) * utilitiesprime[i][j-1]);
          } else if ((i > 0) && (i < 9) && (j == 0)) { //can't go left
            up = (pc * utilitiesprime[i-1][j]) + (((1-pc)/2) * utilitiesprime[i+1][j]) + (((1-pc)/2) * utilitiesprime[i][j+1]);
            left = (pc * utilitiesprime[i][j]) + (((1-pc)/3) * utilitiesprime[i-1][j]) + (((1-pc)/3) * utilitiesprime[i+1][j])
                    + (((1-pc)/3) * utilitiesprime[i][j+1]);
            down = (pc * utilitiesprime[i+1][j]) + (((1-pc)/2) * utilitiesprime[i-1][j]) + (((1-pc)/2) * utilitiesprime[i][j+1]);
            right = (pc * utilitiesprime[i][j+1]) + (((1-pc)/2) * utilitiesprime[i-1][j]) + (((1-pc)/2) * utilitiesprime[i+1][j]);
          } else if ((j > 0) && (j < 39) && (i == 0)) { //can't go up
            up = (pc * utilitiesprime[i][j]) + (((1-pc)/3) * utilitiesprime[i+1][j]) + (((1-pc)/3) * utilitiesprime[i][j+1])
                    + (((1-pc)/3) * utilitiesprime[i][j-1]);
            left = (pc * utilitiesprime[i][j-1]) + (((1-pc)/2) * utilitiesprime[i+1][j]) + (((1-pc)/2) * utilitiesprime[i][j-1]);
            down = (pc * utilitiesprime[i+1][j]) + (((1-pc)/2) * utilitiesprime[i][j+1]) + (((1-pc)/2) * utilitiesprime[i][j-1]);
            right = (pc * utilitiesprime[i][j+1]) + (((1-pc)/2) * utilitiesprime[i+1][j]) + (((1-pc)/2) * utilitiesprime[i][j-1]);
          } else if ((i > 0) && (i < 9) && (j == 39)) { //can't go right
            up = (pc * utilitiesprime[i-1][j]) + (((1-pc)/2) * utilitiesprime[i+1][j]) + (((1-pc)/2) * utilitiesprime[i][j-1]);
            left = (pc * utilitiesprime[i][j-1]) + (((1-pc)/2) * utilitiesprime[i-1][j]) + (((1-pc)/2) * utilitiesprime[i+1][j]);
            down = (pc * utilitiesprime[i+1][j]) + (((1-pc)/2) * utilitiesprime[i-1][j]) + (((1-pc)/2) * utilitiesprime[i][j-1]);
            right = (pc * utilitiesprime[i][j]) + (((1-pc)/3) * utilitiesprime[i-1][j]) + (((1-pc)/3) * utilitiesprime[i][j-1])
                    + (((1-pc)/3) * utilitiesprime[i+1][j]);
          } else if ((j > 0) && (j < 39) && (i == 9)) { //can't go down
            up = (pc * utilitiesprime[i-1][j]) + (((1-pc)/2) * utilitiesprime[i][j-1]) + (((1-pc)/2) * utilitiesprime[i][j+1]);
            left = (pc * utilitiesprime[i][j-1]) + (((1-pc)/2) * utilitiesprime[i-1][j]) + (((1-pc)/2) * utilitiesprime[i][j+1]);
            down = (pc * utilitiesprime[i][j]) + (((1-pc)/3) * utilitiesprime[i-1][j]) + (((1-pc)/3) * utilitiesprime[i][j-1])
                    + (((1-pc)/3) * utilitiesprime[i][j+1]);
            right = (pc * utilitiesprime[i][j+1]) + (((1-pc)/2) * utilitiesprime[i-1][j]) + (((1-pc)/2) * utilitiesprime[i][j-1]);
          } else if (i == 0 && j == 0) { //top right corner, can't go up, can't go left
            up = (pc * utilitiesprime[i][j]) + (((1-pc)/2) * utilitiesprime[i+1][j]) + (((1-pc)/2) * utilitiesprime[i][j+1]);
            left = (pc * utilitiesprime[i][j]) + (((1-pc)/2) * utilitiesprime[i+1][j]) + (((1-pc)/2) * utilitiesprime[i][j+1]);
            down = (pc * utilitiesprime[i+1][j]) + ((1-pc) * utilitiesprime[i][j+1]);
            right = (pc * utilitiesprime[i][j+1]) + ((1-pc) * utilitiesprime[i+1][j]);
          } else if (i == 9 && j == 39) { //bottom left corner, can't go down, can't go right
            up = (pc * utilitiesprime[i-1][j]) + ((1-pc) * utilitiesprime[i][j-1]);
            left = (pc * utilitiesprime[i][j-1]) + ((1-pc) * utilitiesprime[i-1][j]);
            down = (pc * utilitiesprime[i][j]) + (((1-pc)/2) * utilitiesprime[i-1][j]) + (((1-pc)/2) * utilitiesprime[i][j-1]);
            right = (pc * utilitiesprime[i][j]) + (((1-pc)/2) * utilitiesprime[i-1][j]) + (((1-pc)/2) * utilitiesprime[i][j-1]);
          } else if (i == 0 && j == 39) { //top left corner, can't go up, can't go right
            up = (pc * utilitiesprime[i][j]) + (((1-pc)/2) * utilitiesprime[i+1][j]) + (((1-pc)/2) * utilitiesprime[i][j-1]);
            left = (pc * utilitiesprime[i][j-1]) + ((1-pc) * utilitiesprime[i+1][j]);
            down = (pc * utilitiesprime[i+1][j]) + ((1-pc) * utilitiesprime[i][j-1]);
            right = (pc * utilitiesprime[i][j])  + (((1-pc)/2) * utilitiesprime[i][j-1]) + (((1-pc)/2) * utilitiesprime[i+1][j]);
          } else if (i == 9 && j == 0) { //bottom right corner, can't go down, can't go left
            up = (pc * utilitiesprime[i-1][j]) + ((1-pc) * utilitiesprime[i][j+1]);
            left = (pc * utilitiesprime[i][j]) + (((1-pc)/2) * utilitiesprime[i-1][j]) + (((1-pc)/2) * utilitiesprime[i][j+1]);
            down = (pc * utilitiesprime[i][j]) + (((1-pc)/2) * utilitiesprime[i-1][j]) + (((1-pc)/2) * utilitiesprime[i][j+1]);
            right = (pc * utilitiesprime[i][j+1]) + ((1-pc) * utilitiesprime[i-1][j]);
          }
          maxval = getMax(left, right, up, down);
          utilities[i][j] = -0.04 + (discountValue * maxval);
          float temputil = utilitiesprime[i][j] - utilities[i][j];
          //std::cout << "prime: " << utilitiesprime[i][j] << " " << "reg: " << utilities[i][j] << std::endl;
          if (temputil < 0) {
            temputil = temputil * (-1.0);
          }
          if (temputil > maxUtilityChange) {
            maxUtilityChange = temputil;
          }
        }
      }
    }
    // for (int i=0; i<sim1->getHeight(); i++) {
    //   for (int j=0; j<sim1->getWidth(); j++) {
    //     std::cout << utilities[i][j] << " ";
    //   }
    //   std::cout << "|" << std::endl;
    // }
    // std::cout << "max " << maxUtilityChange << std::endl;
  } while (maxUtilityChange > (epsilon * (1 - discountValue))/discountValue);
}

float Project3::getMax(float left, float right, float up, float down) {
  float max = -10000000.0;

  if (left > max) {
    max = left;
  }
  if (right > max) {
    max = right;
  }
  if (up > max) {
    max = up;
  }
  if (down > max) {
    max = down;
  }

  return max;
}

/**
 * @brief get optimal action
 * @param sim simulator pointer
 * @param r robot pointer
 * @return optimal action
 */
RobotAction Project3::getOptimalAction(Simulator* sim1, Robot* r1) {
	// Here, you should find the next step of the robot.
	// The robot should always follow a shortest path (wrt the known and sensed obstacles) to the goal.
    for (int i=0; i<sim1->getHeight(); i++) {
      for (int j=0; j<sim1->getWidth(); j++) {
        std::cout << arrow[i][j] << " ";
      }
      std::cout << std::endl;
    }
    return directions[(int)r1->getPosition().x][(int)r1->getPosition().y];
}
