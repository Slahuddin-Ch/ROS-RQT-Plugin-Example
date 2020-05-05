
#ifndef qtros_MAIN_WINDOW_H
#define qtros_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qtros {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();


	void showSetMessage();
	void showCantSetMessage();
	


public Q_SLOTS:

	void on_set_clicked(bool check );   // set number 
	void on_pushButton_clicked(bool check); // reset output
	void on_pushButton_2_clicked(bool check); //reset logging
	void setStatusON(); // set status
	void setStatusOFF(); // set status
	

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace qtros

#endif // qtros_MAIN_WINDOW_H

