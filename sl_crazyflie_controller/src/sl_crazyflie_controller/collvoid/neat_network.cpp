#include "../../../include/neat_network.h"
#include <fstream>
#include <iostream>
#include "std_msgs/String.h"
#include "sl_crazyflie_controller/GetNEATOutputs.h"


NEATNode::NEATNode(std::string gf) : genomeFile(gf) {

   //Take in network from file
   readFile();

   //Get network from organism
   org = new NEAT::Organism(0.0, genom, 1);
   net = org->net;

   //Create publisher to be able to write to topic 'net_outputs'
   //net_outputs_pub = n.advertise<std_msgs::String>("net_outputs", 1000);

}

//Read weights of network from file
void NEATNode::readFile() {

   char curword[20];
   int id;

   std::ifstream iFile(genomeFile.c_str(), std::ios::in);

   std::cout << "Reading in the individual.." << std::endl;

   iFile >> curword;
   iFile >> id;

   // if(!iFile)
	// {
	// 	std::cout<< "Can't find source file" << std::endl;
   //
	// } else {
   //
   //    std::cout<< "Source file found!" << std::endl;
   // }

   genom = new NEAT::Genome(id, iFile);

   iFile.close();

   std::cout << "File read!" << std::endl;

}

bool propogate(sl_crazyflie_controller::GetNEATOutputs::Request  &req,
               sl_crazyflie_controller::GetNEATOutputs::Response &res)
    {

      res.output1 = req.input1 + req.input2;
      res.output2 = req.input1 - req.input2;

      return true;
  }

void NEATNode::run() {

   // ros::Rate loop_rate(1);
   //
   // while(ros::ok()) {
   //
   //    std_msgs::String msg;
   //
   //    std::stringstream ss;
   //    ss << "hello world ";
   //    msg.data = ss.str();
   //
   //    net_outputs_pub.publish(msg);
   //
   //    ros::spinOnce();
   //
   //    loop_rate.sleep();
   //
   // }


  ros::ServiceServer service = n.advertiseService("get_neat_outputs", propogate);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

}

int main(int argc, char **argv) {

   ros::init(argc, argv, "neat_net_node");

   //Genome file as argument
   NEATNode neat_node = NEATNode("/home/james/catkin_ws/src/sl_crazyflie_dwm1000/sl_crazyflie_controller/src/sl_crazyflie_controller/collvoid/genomes/overall_winner_1");
   neat_node.run();

}
