#pragma execution_character_set("utf-8")
#ifndef _FILTERSTATISTICAL_H
#define _FILTERSTATISTICAL_H

#include <QDialog>
#include "ui_FilterStatistical.h"

class FilterStatistical : public QDialog
{
	Q_OBJECT
public:
	FilterStatistical(QWidget* parent = nullptr);
	~FilterStatistical();

signals:
	void sendData(QString data_1, QString data_2);
private slots:
	void btn_Ok_clicked();
	void btn_Cancel_clicked();
private:
	Ui::FilterStatistical* ui;
};

#endif