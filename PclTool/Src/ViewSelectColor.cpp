#include "ViewSelectColor.h"
#include <QHBoxLayout>
#include <QColorDialog>
#include <QIcon>

ViewSelectColor::ViewSelectColor()
{
	this->setWindowIcon(QIcon(":/Res/Res/BackGround.png"));
	QColor color = QColorDialog::getColor(Qt::white, this);

	if (color.isValid()) {
		m_color = color;
	}
	else {
		color.setRgb(255, 255, 255, 255);
		m_color = color;
	}
}

ViewSelectColor::~ViewSelectColor()
{
}

void ViewSelectColor::setColor(const QColor& color)
{
	m_color = color;
}

QColor ViewSelectColor::getColor()
{
	return m_color;
}
