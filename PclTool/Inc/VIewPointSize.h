#pragma execution_character_set("utf-8")
#ifndef _VIEWPOINTSIZE_H
#define _VIEWPOINTSIZE_H

#include <QDialog>
#include "ui_ViewPointSize.h"

class ViewPointSize : public QDialog
{
	Q_OBJECT

public:
	explicit ViewPointSize(QWidget* parent = nullptr);
	~ViewPointSize();

signals:
	void sendData(QString data);

private slots:
	void btn_Ok_clicked();
	void btn_Cancel_clicked();

private:
	Ui::ViewPointSize* ui;
};

#endif