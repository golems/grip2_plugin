/********************************************************************************
** Form generated from reading UI file 'planningtab.ui'
**
** Created: Tue Apr 8 09:43:45 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PLANNINGTAB_H
#define UI_PLANNINGTAB_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDockWidget>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "../include/qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_PlotTabWidget
{
public:
    QWidget *dockWidgetContents;
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QCustomPlot *customPlot;

    void setupUi(QDockWidget *PlotTabWidget)
    {
        if (PlotTabWidget->objectName().isEmpty())
            PlotTabWidget->setObjectName(QString::fromUtf8("PlotTabWidget"));
        PlotTabWidget->resize(928, 505);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        gridLayout = new QGridLayout(dockWidgetContents);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        customPlot = new QCustomPlot(dockWidgetContents);
        customPlot->setObjectName(QString::fromUtf8("customPlot"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(customPlot->sizePolicy().hasHeightForWidth());
        customPlot->setSizePolicy(sizePolicy);

        verticalLayout->addWidget(customPlot);


        horizontalLayout->addLayout(verticalLayout);


        gridLayout->addLayout(horizontalLayout, 0, 1, 1, 1);

        PlotTabWidget->setWidget(dockWidgetContents);

        retranslateUi(PlotTabWidget);

        QMetaObject::connectSlotsByName(PlotTabWidget);
    } // setupUi

    void retranslateUi(QDockWidget *PlotTabWidget)
    {
        PlotTabWidget->setWindowTitle(QApplication::translate("PlotTabWidget", "Plot Tab", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PlotTabWidget: public Ui_PlotTabWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PLANNINGTAB_H
