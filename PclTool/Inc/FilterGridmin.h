#pragma execution_character_set("utf-8")
#ifndef _FILTERGRIDMIN_H
#define _FILTERGRIDMIN_H

#include <QDialog>
#include "ui_FilterGridmin.h"

class FilterGridmin : public QDialog
{
	Q_OBJECT
public:
	explicit FilterGridmin(QWidget *parent = nullptr);
	~FilterGridmin();

signals:
	void sendData(QString data);

private slots:
	void btn_Ok_clicked();
	void btn_Cancel_clicked();

private:
	Ui::FilterGridmin* ui;
};

#endif