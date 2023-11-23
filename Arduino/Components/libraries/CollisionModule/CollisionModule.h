#ifndef COLLISION_MODULE_H_INCLUDED
#define COLLISION_MODULE_H_INCLUDED

#include <Arduino.h>

class CollisionModule {
  private:
    int* _sensorPins;
    int _sensorStates[6];
    
  public:
    // Constructor
    CollisionModule(int* sensorPins);

    // Method to check for almost collision
    void checkCollision();
    int getMovementInstructions();
};

#endif // COLLISION_MODULE_H_INCLUDED