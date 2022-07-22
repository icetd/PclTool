#include "FilterUniform.h"

FilterUniform::FilterUniform(QWidget* parent) :
	QDialog(parent),
	ui(new Ui::FilterUniform)
{
	ui->setupUi(this);
	this->setWindowTitle("¾ùÔÈ²ÉÑù");
	this->setWindowIcon(QIcon("/Res/Res/Filter.png"));
}

FilterUniform::~FilterUniform()
{
	delete ui;
}

void FilterUniform::btn_Ok_clicked()
{
	emit sendData(ui->lineEdit->text());
	this->close();
}

void FilterUniform::btn_Cancel_clicked()
{
	this->close();
}
