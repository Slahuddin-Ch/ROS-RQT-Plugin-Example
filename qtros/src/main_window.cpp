
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/qtros/main_window.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{

	qnode.init();


	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	QObject::connect(&qnode, SIGNAL(nodeIsActive()), this, SLOT(setStatusON()));
	QObject::connect(&qnode, SIGNAL(nodeIsNotActive()), this, SLOT(setStatusOFF()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
	ui.view_output->setModel(qnode.outputModel());

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

// set limit for max number
void MainWindow::on_set_clicked(bool check ) {
	
	showSetMessage();
}

// reset output
void MainWindow::on_pushButton_clicked(bool check) {
	
	qnode.clearOutput();
}

// set ros node status on
void MainWindow::setStatusON()
{
	ui.Label_RosNodeStatus->setText("ON");
}

// set ros node status off
void MainWindow::setStatusOFF()
{
	ui.Label_RosNodeStatus->setText("OFF");
}

// reset log output
void MainWindow::on_pushButton_2_clicked(bool check) {
	
	qnode.clearLogging();
}

// to check if the comming number is integer
bool is_number(const std::string& s)
{
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
}

// display message
void MainWindow::showSetMessage() {
	QMessageBox msgBox;
	msgBox.setText("change size");
	msgBox.exec();

	std::string labelValue = ui.line_value->text().toStdString();
	if (!(is_number(labelValue)))
	{
		showCantSetMessage();
	}
	else
	{
		qnode.changeMax(std::stoi(labelValue));
	}

}
// if the input in max number is not a number
void MainWindow::showCantSetMessage() {
	QMessageBox msgBox;
	msgBox.setText("cant set this value");
	msgBox.exec();

}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/



}  // namespace qtros

