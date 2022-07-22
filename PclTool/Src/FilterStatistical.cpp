#include "FilterStatistical.h"

FilterStatistical::FilterStatistical(QWidget* parent) :
	QDialog(parent),
	ui(new Ui::FilterStatistical)
{
	ui->setupUi(this);
	this->setWindowTitle("Í³¼ÆÂË²¨");
	this->setWindowIcon(QIcon("/Res/Res/Filter.png"));
	ui->lineEdit->setText("1");
}

FilterStatistical::~FilterStatistical()
{
	delete ui;
}

void FilterStatistical::btn_Ok_clicked()
{
	emit sendData(ui->lineEdit->text(), ui->lineEdit_2->text());
	this->close();
}

void FilterStatistical::btn_Cancel_clicked()
{
	this->close();
}
