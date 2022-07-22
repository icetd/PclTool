#include "FilterGridmin.h"

FilterGridmin::FilterGridmin(QWidget* parent) :
	QDialog(parent),
	ui(new Ui::FilterGridmin)
{
	ui->setupUi(this);
	this->setWindowTitle("GridMin²ÉÑù");
	this->setWindowIcon(QIcon("/Res/Res/Filter.png"));
}

FilterGridmin::~FilterGridmin()
{
	delete ui;
}

void FilterGridmin::btn_Ok_clicked()
{
	emit sendData(ui->lineEdit->text());
	this->close();
}

void FilterGridmin::btn_Cancel_clicked()
{
	this->close();
}