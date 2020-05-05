

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/qtros/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	nPinPoint = 0;  // start uploading list
	nRecievedValue1 , nRecievedValue2 = 1.0;   // to calculate ratio
	recievedData = 0; // no to recieve
	maxNumber = 100;  //Max Number
	nodeStatus = 0;   //node status variable
	
	ros::init(init_argc,init_argv,"qtros");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	start();
	// ros communications
	sub = n.subscribe("chatter", 10000, &QNode::chatterCallback, this);
	chatter1_pub = n.advertise<std_msgs::String>("chatter1", 1000);
	return true;
}

bool QNode::init(int argc, char** argv ) {
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	//ros communications	
	chatter1_pub = n.advertise<std_msgs::String>("chatter1", 1000);
	sub = n.subscribe("chatter", 1000, &QNode::chatterCallback, this);
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(0.5);
	
	while ( ros::ok() ) {
		
		//if node is not responding mark it off
		int responseTime = 2; 
		if (nodeStatus > responseTime){
			Q_EMIT nodeIsNotActive();
		}

		// if number is larger that max
    	if (recievedData > maxNumber){
			nPinPoint = 0;
			log(Info,std::string("\n\n\n------------------------ Recieved No. is bigger ---------------------------\n\n\n"));
		    std_msgs::String msgBack;
		    std::stringstream ss;
      		ss << 1;
      		msgBack.data = ss.str();
      		chatter1_pub.publish(msgBack);
			log(Info,std::string("\n\n\n------------------------ Sending Reset Request ---------------------------\n\n\n"));
      		sleep(2);
    	}
		nodeStatus++; //if node doesnot respend in 5 loods status will be set off.
		ros::spinOnce();
    	loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

// to display log
void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO]: I recieved -->  " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[WARN]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

// cleat output
void QNode::clearOutput()
{
	output_model.setStringList( QStringList{} );
}

// clear log
void QNode::clearLogging()
{
	logging_model.setStringList( QStringList{} );
}

void QNode::displayUI(const std::string &msg) {
	output_model.setStringList( QStringList{} );
	output_model.insertRows(output_model.rowCount(),1);
	std::stringstream output_model_msg;
	ROS_INFO_STREAM(msg);
	output_model_msg << "LIST --> " << msg;
	QVariant new_row(QString(output_model_msg.str().c_str()));
	output_model.setData(output_model.index(output_model.rowCount()-1) , new_row);
	Q_EMIT outputUpdated(); // used to readjust the scrollbar
}


void QNode::chatterCallback(const std_msgs::String::ConstPtr& msg)
{
   //recieve message
   std::cout<< msg->data.c_str();
   
   // display on log
   log(Info,std::string(msg->data.c_str()));

   // emmit response that node is working
   Q_EMIT nodeIsActive();
   nodeStatus = 0;

   // if recieved number is within limits i.e not < 0 or infinite
   int number = std::stoi(msg->data.c_str());
   if (!isfinite(number) || number < 0)
   {
	   recievedData = 0;
   }
   else
   {
	   recievedData = number;
   }
	double ratio = 1;
	//if recieved data is zero then do not cakculate ratio
	if (recievedData){
		if (nRecievedValue2 > nRecievedValue1){
      		nRecievedValue1 = nRecievedValue2;
      		nRecievedValue2 = recievedData;
      		ratio = (nRecievedValue2 / nRecievedValue1);
   		}
		else{
      		nRecievedValue2 = nRecievedValue1;
      		nRecievedValue1 = recievedData;
      		ratio = (nRecievedValue1 / nRecievedValue2);
   		}
	}
	//add ratio to list
	if (nPinPoint < nList.size()){
		nList[nPinPoint] = ratio;
		nPinPoint++;
	}
	else{
		nList.push_back(ratio);
		nPinPoint++;
	}
	std::string display = " ";
   	for (int i = 0; i < nList.size() ; i++ ){
	 	display += std::to_string(nList[i]);
	 	display += "   ";
   	}
   	displayUI(display);
}

void QNode::changeMax(int set){
	maxNumber = set;
}

}  // namespace qtros

