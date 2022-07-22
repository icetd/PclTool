#ifndef _VIEWSELECTCOLOR_H
#define _VIEWSELECTCOLOR_H

#include <QWidget>


class ViewSelectColor : public QWidget
{
	Q_OBJECT
public:
	ViewSelectColor();
	~ViewSelectColor();

	void setColor(const QColor& color);
	QColor getColor();

private:
	QColor m_color;
};

#endif
