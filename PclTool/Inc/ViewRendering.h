#pragma execution_character_set("utf-8")
#ifndef _VIEWRENDERING_H
#define _VIEWRENDERING_H

#include <QDialog>
#include "ui_ViewRendering.h"

class ViewRendering : public QDialog
{
	Q_OBJECT
public:
	explicit ViewRendering(QWidget* parent = nullptr);
	~ViewRendering();

signals:
	void sendData(QString data);

private slots:
	
	void btn_Ok_clicked();
	void btn_Cancel_clicked();
	void btn_x_clicked();
	void btn_y_clicked();
	void btn_z_clicked();
	void btn_i_clicked();

private:
	Ui::ViewRendering *ui;
	QString axis;
};

#endif