#include "FilterAvoxel.h"

FilterAvoxel::FilterAvoxel(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::FilterAvoxel)
{
	ui->setupUi(this);
	this->setWindowTitle("½üËÆÂË²¨");
	this->setWindowIcon(QIcon("/Res/Res/Filter.png"));
}

FilterAvoxel::~FilterAvoxel()
{
	delete ui;
}

void FilterAvoxel::btn_Ok_clicked()
{
	emit sendData(ui->lineEdit->text());
	this->close();
}

void FilterAvoxel::btn_Cancel_clicked()
{
	this->close();
}
