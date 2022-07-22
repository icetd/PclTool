#include "VIewPointSize.h"

ViewPointSize::ViewPointSize(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::ViewPointSize)
{
	ui->setupUi(this);
	this->setWindowTitle("µãÔÆ´óÐ¡");
	this->setWindowIcon(QIcon(":/Res/Res/PointSize.png"));

	ui->spinBox_size->setMinimum(1);
	ui->spinBox_size->setMaximum(50);
	ui->spinBox_size->setSingleStep(1);
	ui->spinBox_size->setValue(1);

	ui->h_Slider_size->setMinimum(1);
	ui->h_Slider_size->setMaximum(50);
	ui->h_Slider_size->setSingleStep(1);
	ui->h_Slider_size->setValue(1);

	connect(ui->spinBox_size, SIGNAL(valueChanged(int)), ui->h_Slider_size, SLOT(setValue(int)));
	connect(ui->h_Slider_size, SIGNAL(valueChanged(int)), ui->spinBox_size, SLOT(setValue(int)));
}

ViewPointSize::~ViewPointSize()
{
	delete ui;
}

void ViewPointSize::btn_Ok_clicked()
{
	QString value = QString("%1").arg(ui->h_Slider_size->value());
	emit sendData(value);
	this->close();
}

void ViewPointSize::btn_Cancel_clicked()
{
	this->close();
}

