#include "../../../include/neat_network.h"
#include <fstream>
#include <iostream>
#include "std_msgs/String.h"

#include "std_msgs/Float64MultiArray.h"

NEATNode::NEATNode(std::string gf) : genomeFile(gf),
                                     NUM_DRONES(2),
                                     theta(0.0) {

   //Take in network from file
   readFile();

   //Get network from organism
   org = new NEAT::Organism(0.0, genom, 1);
   NEATnet = org->net;

   net_inputs.resize(NEATnet->inputs.size());
   net_outputs.resize(NEATnet->outputs.size());

   net_inputs[0] = 1.0;                            //Bias node

   //Initialise inputs
   for(int i = 1; i < NEATnet->inputs.size(); i++) {

      net_inputs[i] = 1.0;

   }

   //Create publisher to be able to write to topic 'net_outputs'
   net_outputs_pub = n.advertise<std_msgs::Float64MultiArray>("NEAT_outputs", 1000);

   //Distance topic Subscriber
   distance_sub = n.subscribe("log_ranges", 1000, &NEATNode::receivedDistance, this);

   n.getParam("flight_controller/collvoid/geofencing/max_x", max_wall_x);
   n.getParam("flight_controller/collvoid/geofencing/max_y", max_wall_y);
   n.getParam("flight_controller/collvoid/geofencing/min_x", min_wall_x);
   n.getParam("flight_controller/collvoid/geofencing/min_y", min_wall_y);

   // if (n.getParam("flight_controller/collvoid/geofencing/max_x", max_wall_x)) {
   //    std::cout << "Waayyayyoo" << std::endl;
   //    std::cout << max_wall_x << std::endl;
   // } else {
   //    std::cout << "Could not get parameter" << std::endl;
   // }

   // std::cout << min_wall_x << std::endl;
   // std::cout << min_wall_y << std::endl;
   // std::cout << max_wall_x << std::endl;
   // std::cout << max_wall_y << std::endl;

   //Position topic Subscriber
   //Use param from cf.launch

   std::string pose_topic;
   n.getParam("flight_controller/cf_pose_topic", pose_topic);
   //std::cout << "Pose topic: " << pose_topic << std::endl;

   //wall_sub = n.subscribe("/Robot_1/pose", 1000, &NEATNode::receivedWallDist, this);
   wall_sub = n.subscribe(pose_topic, 1000, &NEATNode::receivedWallDist, this);

}

//Called when log_ranges has a new input - the distance to the other Crazyflies
void NEATNode::receivedDistance(const crazyflie_driver::GenericLogData& input) {

   // std::cout << input.values[0] << std::endl;
   //
   std::vector<double> distances(NUM_DRONES-1);

   for(int i = 0; i < (NUM_DRONES-1); i++) {

      distances[i] = input.values[i];

   }

   double min_dist = *std::min_element(distances.begin(), distances.end());
   //std::cout << "Min Dist: " << min_dist << std::endl;

   //net_inputs[1] = min_dist;
   net_inputs[1] = 1.0;

}

//Actually this just receives the pose, but it is what I use to calculate the
//distance to the walls
void NEATNode::receivedWallDist(const geometry_msgs::PoseStamped& input) {

   //std::cout << input.pose.position.x << std::endl;

   std::vector<double> wall_distances(4);

   wall_distances[0] = std::abs(max_wall_x - input.pose.position.x);    //North wall (door)
   wall_distances[1] = std::abs(min_wall_x - input.pose.position.x);    //South wall (window)
   wall_distances[2] = std::abs(max_wall_y - input.pose.position.y);    //West wall (couch)
   wall_distances[3] = std::abs(min_wall_y - input.pose.position.y);    //East wall (computers)

   double closest_wall = *min_element(wall_distances.begin(), wall_distances.end());

   //std::cout << closest_wall << std::endl;
   //std::cout << max_wall_y << std::endl;

   net_inputs[2] = closest_wall;

}

//Publishes outputs to
void NEATNode::publishOutputs() {

   std_msgs::Float64MultiArray outs;

   outs.data.clear();

   for(int i = 0; i < net_outputs.size(); i++) {

      outs.data.push_back(net_outputs[i]);

   }

   net_outputs_pub.publish(outs);

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

//Propogates the service inputs through the network and returns
//outputs
// bool propogate(sl_crazyflie_controller::GetNEATOutputs::Request  &req,
//                sl_crazyflie_controller::GetNEATOutputs::Response &res)
//    {
//
//       //Create inputs for the NEAT neural network
//       //The number of sensors plus a bias node
//       std::vector<float> inputs(NEATnet->inputs.size());
//       double outputs[NEATnet->outputs.size()];
//
//       inputs[0] = 1.0;      //Bias
//
//       inputs[1] = req.input1;   //Closest drone
//       inputs[2] = req.input2;   //Closest wall
//
//       //Load input sensors
//       NEATnet->load_sensors(inputs);
//
//       //Activate network
//       //If it loops, will return false
//       if (!(NEATnet->activate())) std::cout << "Inputs disconnected from output!";
//
//       //Get outputs
//       std::vector<NEAT::NNode*>::iterator it;
//
//       for(it = NEATnet->outputs.begin(); it != NEATnet->outputs.end(); it++) {
//
//          outputs[it-NEATnet->outputs.begin()] = (*it)->activation;
//
//       }
//
//       // res.output1 = req.input1;
//       // res.output2 = req.input2;
//
//       // res.output1 = outputs[0];
//       // res.output2 = outputs[1];
//
//       res.output1 = NEATNode::MapValueIntoActuatorRange(outputs[0]);
//       res.output2 = NEATNode::MapValueIntoActuatorRange(outputs[1]);
//
//       return true;
//    }

std::vector<double> NEATNode::propogate(std::vector<float> inputs) {

   //std::cout << inputs[1] << " " << inputs[2] << std::endl;
   //std::cout << inputs[2] << std::endl;

   //Load input sensors
   NEATnet->load_sensors(inputs);

   //Activate network
   //If it loops, will return false
   if (!(NEATnet->activate())) std::cout << "Inputs disconnected from output!";

   //Get outputs
   std::vector<double> outputs(NEATnet->outputs.size());
   std::vector<NEAT::NNode*>::iterator it;

   //Process outputs
   for(it = NEATnet->outputs.begin(); it != NEATnet->outputs.end(); it++) {

      outputs[it-NEATnet->outputs.begin()] = MapValueIntoActuatorRange((*it)->activation);
      //std::cout << outputs[it-NEATnet->outputs.begin()] << " ";

   }

   //std::cout << std::endl;

   std::vector<double> speeds(outputs.size());

   speeds = GetSpeedsForActuator(outputs[0], outputs[1]);

   //std::cout << "Theta: " << theta << std::endl;

   //return outputs;
   return speeds;

}

double NEATNode::MapValueIntoActuatorRange(double value) {

   double nn_span = nn_output_ub - nn_output_lb;
   double actuator_span = rotor_actuator_ub - rotor_actuator_lb;

   double norm = (value - nn_output_lb) / nn_span;

   return (norm * actuator_span) + rotor_actuator_lb;

}

std::vector<double> NEATNode::GetSpeedsForActuator(double left_speed, double right_speed) {

   /* Convert left and right speeds to linVel and rotVel */

   double linVel = (left_speed + right_speed) * 0.5;
   double rotVel = (left_speed - right_speed) / 1.0;

   /* Arrow method */

   //Update theta
   theta = theta + (rotVel/rotScalingFactor);

   //Calculate speeds based on arrow
   double xSpeed = linVel * cos(theta);
   double ySpeed = linVel * sin(theta);
   double zSpeed = 0;

   //  std::cout << "xSpeed: " << xSpeed << std::endl;
   //  std::cout << "ySpeed: " << ySpeed << std::endl;

   std::vector<double> speeds;
   speeds.push_back(xSpeed);
   speeds.push_back(ySpeed);
   speeds.push_back(zSpeed);

   return speeds;

}

//Main looping function
void NEATNode::run() {

   ros::Rate loop_rate(200);

   while(ros::ok()) {

      //std::cout << "Propogate" << std::endl;

      //Propogate inputs through the network
      net_outputs = propogate(net_inputs);

      publishOutputs();

      loop_rate.sleep();

      //Allows callbacks to happen
      ros::spinOnce();

   }

   //Create ROS service
   // ros::ServiceServer service = n.advertiseService("get_neat_outputs", propogate);
   // ROS_INFO("Network ready to receive input!");
   // ros::spin();

}

int main(int argc, char **argv) {

   ros::init(argc, argv, "neat_net_node");

   //Genome file as argument
   NEATNode neat_node = NEATNode("/home/james/catkin_ws/src/sl_crazyflie_dwm1000/sl_crazyflie_controller/src/sl_crazyflie_controller/collvoid/genomes/overall_winner_1");
   neat_node.run();

}
