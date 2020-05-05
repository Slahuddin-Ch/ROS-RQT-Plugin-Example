
#ifndef qtros_QNODE_HPP_
#define qtros_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <vector> 


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(int argc, char** argv );
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	QStringListModel* outputModel() { return &output_model; }
	void log( const LogLevel &level, const std::string &msg);
	void displayUI(const std::string &msg);
	void changeMax(int set);
	void clearOutput();
	void clearLogging();


	void chatterCallback(const std_msgs::String::ConstPtr& msg);

Q_SIGNALS:
	void loggingUpdated(); // to update log
	void outputUpdated();  // to update output
    void rosShutdown();    // to shutdown ros
	void nodeIsActive();  // to keep checking if node is still active.
	void nodeIsNotActive();  // to keep checking if node is still active.

private:
	int init_argc;
	char** init_argv;
	
	//communication
	ros::Publisher chatter1_pub;
	ros::Subscriber sub;
    QStringListModel logging_model;
	QStringListModel output_model;

	//variables
	std::vector<double> nList;   
	int nPinPoint;
	double nRecievedValue1 , nRecievedValue2;
	int recievedData;
	int maxNumber;
	int nodeStatus;
};

}  // namespace qtros

#endif /* qtros_QNODE_HPP_ */
