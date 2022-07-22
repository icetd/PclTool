#include "FilterRandom.h"

FilterRandom::FilterRandom(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::FilterRandom)
{
	ui->setupUi(this);
	this->setWindowTitle("¹Ì¶¨²ÉÑù");
	this->setWindowIcon(QIcon("/Res/Res/Filter.png"));
}

FilterRandom::~FilterRandom()
{
	delete ui;
}

void FilterRandom::btn_Ok_clicked()
{
	emit sendData(ui->lineEdit->text());
	this->close();
}

void FilterRandom::btn_Cancel_clicked()
{
	this->close();
}
