#pragma execution_character_set("utf-8")
#ifndef _FILTERRANDOM_H
#define _FILTERRANDOM_H

#include <QDialog>
#include "ui_FilterRandom.h"

class FilterRandom : public QDialog
{
	Q_OBJECT

public:
	explicit FilterRandom(QWidget* parent = nullptr);
	~FilterRandom();

signals:
	void sendData(QString data);

private slots:
	void btn_Ok_clicked();
	void btn_Cancel_clicked();

private:
	Ui::FilterRandom* ui;
};

#endif