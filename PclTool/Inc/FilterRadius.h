#pragma execution_character_set("utf-8")
#ifndef _FILTERRADIUS_H
#define _FILTERRADIUS_H

#include <QDialog>
#include "ui_FilterRadius.h"

class FilterRadius : public QDialog
{
	Q_OBJECT
public:
	explicit FilterRadius(QWidget* parent = nullptr);
	~FilterRadius();

signals:
	void sendData(QString data_1, QString data_2);

private slots:
	void btn_Ok_clicked();
	void btn_Cancel_clicked();

private:
	Ui::FilterRadius* ui;
};

#endif