#include "FilterVoxel.h"

FilterVoxel::FilterVoxel(QWidget* parent) :
	QDialog(parent),
	ui(new Ui::FilterVoxel)
{
	ui->setupUi(this);
	this->setWindowTitle("ÌåËØÂË²¨");
	this->setWindowIcon(QIcon(":/Res/Res/Filter.png"));
}

FilterVoxel::~FilterVoxel()
{
	delete ui;
}

void FilterVoxel::btn_Ok_clicked()
{
	emit sendData(ui->lineEdit->text());
	this->close();
}

void FilterVoxel::btn_Cancel_clicked()
{
	this->close();
}