#ifndef NEAT_NODE_H
#define NEAT_NODE_H

#include "ros/ros.h"

#include "../NEAT/include/genome.h"
#include "../NEAT/include/organism.h"
#include "../NEAT/include/network.h"

class NEATNode {

public:

   NEATNode(std::string gf);

   void run();


private:

   void readFile();

   NEAT::Genome* genom;
   NEAT::Organism* org;
   NEAT::Network* net;

   std::string genomeFile;

   //Publisher object for outputs
   ros::Publisher net_outputs_pub;

   //Handle for the node
   ros::NodeHandle n;

};

#endif
