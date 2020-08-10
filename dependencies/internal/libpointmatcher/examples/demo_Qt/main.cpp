#include <QApplication>
#include <QWidget>
#include "PointMatcher.h"

class LAUPointMatcherWidget : public QWidget
{
    //Q_OBJECT

	public:
        //LAUPointMatcherWidget(QWidget *parent = 0) : QWidget(parent) { ; }
        //~LAUPointMatcherWidget();

	private:
        PointMatcher<float>::DataPoints::Label label;
};

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
	LAUPointMatcherWidget w;
    w.show();

    return a.exec();
}

