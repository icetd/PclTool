#pragma execution_character_set("utf-8")
#ifndef _FILTERVOXEL_H
#define _FILTERVOXEL_H

#include <QDialog>
#include "ui_FilterVoxel.h"

class FilterVoxel : public QDialog
{
	Q_OBJECT
public:
	explicit FilterVoxel(QWidget *parent = nullptr);
	~FilterVoxel();

signals:
	void sendData(QString data);

private slots:
	void btn_Ok_clicked();
	void btn_Cancel_clicked();

private:
	Ui::FilterVoxel* ui;
};

#endif 
