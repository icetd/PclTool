#include "ViewRendering.h"
#include <QMessageBox>

ViewRendering::ViewRendering(QWidget* parent) :
	QDialog(parent),
	ui(new Ui::ViewRendering)
{
	ui->setupUi(this);
	this->setWindowTitle("µãÔÆäÖÈ¾");
	this->setWindowIcon(QIcon(":/Res/Res/CloudRender.png"));
}

ViewRendering::~ViewRendering()
{
	delete ui;
}

void ViewRendering::btn_Cancel_clicked()
{
	this->close();
}

void ViewRendering::btn_Ok_clicked()
{
	emit sendData(axis);
	this->close();
}

void ViewRendering::btn_x_clicked()
{
	if (ui->btn_x->isChecked()) {
		ui->btn_y->setChecked(false);
		ui->btn_z->setChecked(false);
		ui->btn_i->setChecked(false);
		axis = "x";
	}
}

void ViewRendering::btn_y_clicked()
{
	if (ui->btn_y->isChecked()) {
		ui->btn_x->setChecked(false);
		ui->btn_z->setChecked(false);
		ui->btn_i->setChecked(false);
		axis = "y";
	}
}

void ViewRendering::btn_z_clicked()
{
	if (ui->btn_z->isChecked()) {
		ui->btn_x->setChecked(false);
		ui->btn_y->setChecked(false);
		ui->btn_i->setChecked(false);
		axis = "z";
	}
}

void ViewRendering::btn_i_clicked()
{
	if (ui->btn_i->isChecked()) {
		ui->btn_x->setChecked(false);
		ui->btn_y->setChecked(false);
		ui->btn_z->setChecked(false);
		axis = "intensity";
	}
}

