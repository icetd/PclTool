#pragma execution_character_set("utf-8")
#ifndef _FILTERAVOXEL_H
#define _FILTERAVOXEL_H

#include <QDialog>
#include "ui_FilterAvoxel.h"

class FilterAvoxel : public QDialog
{
	Q_OBJECT

public:
	explicit FilterAvoxel(QWidget *parent = nullptr);
	~FilterAvoxel();

signals:
	void sendData(QString data);

private slots:
	void btn_Ok_clicked();
	void btn_Cancel_clicked();

private:
	Ui::FilterAvoxel *ui;
};

#endif