#ifndef NEAT_NODE_H
#define NEAT_NODE_H

#include "ros/ros.h"

#include "../NEAT/include/genome.h"
#include "../NEAT/include/organism.h"
#include "../NEAT/include/network.h"

#include "crazyflie_driver/GenericLogData.h"

class NEATNode {

public:

   NEATNode(std::string gf);

   void run();

   static double MapValueIntoActuatorRange(double value);

   void receivedDistance(const crazyflie_driver::GenericLogData& input);

private:

   void readFile();

   std::vector<double> propogate(std::vector<float> inputs);

   void publishOutputs();


   NEAT::Genome* genom;
   NEAT::Organism* org;
   NEAT::Network* NEATnet;

   std::string genomeFile;

   //Publisher object for outputs
   ros::Publisher net_outputs_pub;

   //Subscriber for distances
   ros::Subscriber distance_sub;

   //Handle for the node
   ros::NodeHandle n;

   //[1] distance to closest drone
   //[2] distance to closest wall
   std::vector<float> net_inputs;

   std::vector<double> net_outputs;

   static const double nn_output_lb = 0.0;
   static const double nn_output_ub = 1.0;
   static const double rotor_actuator_lb = -0.1;
   static const double rotor_actuator_ub = 0.1;

   //float distance;
   //float position;

   const int NUM_DRONES;

};

#endif
