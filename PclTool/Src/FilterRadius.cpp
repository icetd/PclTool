#include "FilterRadius.h"

FilterRadius::FilterRadius(QWidget* parent) :
	QDialog(parent),
	ui(new Ui::FilterRadius)
{
	ui->setupUi(this);
	this->setWindowTitle("°ë¾¶ÂË²¨");
	this->setWindowIcon(QIcon("/Res/Res/Filter.png"));
}

FilterRadius::~FilterRadius()
{
	delete ui;
}

void FilterRadius::btn_Ok_clicked()
{
	emit sendData(ui->lineEdit_1->text(), ui->lineEdit_2->text());
	this->close();
}

void FilterRadius::btn_Cancel_clicked()
{
	this->close();
}