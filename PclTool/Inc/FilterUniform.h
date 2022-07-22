#pragma execution_character_set("utf-8")
#ifndef _FILTERUNIFORM_H
#define _FILTERUNIFORM_H

#include <QDialog>
#include "ui_FilterUniform.h"

class FilterUniform : public QDialog
{
	Q_OBJECT
public:
	explicit FilterUniform(QWidget *parent = nullptr);
	~FilterUniform();
signals:
	void sendData(QString data);

private slots:
	void btn_Ok_clicked();
	void btn_Cancel_clicked();

private:
	Ui::FilterUniform* ui;
};

#endif