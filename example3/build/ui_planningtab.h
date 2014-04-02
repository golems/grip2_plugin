/********************************************************************************
** Form generated from reading UI file 'planningtab.ui'
**
** Created: Wed Apr 2 14:29:09 2014
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
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PlanningTabWidget
{
public:
    QWidget *dockWidgetContents;
    QGridLayout *gridLayout;
    QPushButton *setStart;
    QPushButton *setGoal;
    QPushButton *setPredefStart;
    QPushButton *setPredefGoal;
    QPushButton *relocateObjects;
    QPushButton *showStart;
    QPushButton *showGoal;
    QPushButton *doPlan;

    void setupUi(QDockWidget *PlanningTabWidget)
    {
        if (PlanningTabWidget->objectName().isEmpty())
            PlanningTabWidget->setObjectName(QString::fromUtf8("PlanningTabWidget"));
        PlanningTabWidget->resize(663, 182);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        gridLayout = new QGridLayout(dockWidgetContents);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        setStart = new QPushButton(dockWidgetContents);
        setStart->setObjectName(QString::fromUtf8("setStart"));

        gridLayout->addWidget(setStart, 0, 0, 1, 1);

        setGoal = new QPushButton(dockWidgetContents);
        setGoal->setObjectName(QString::fromUtf8("setGoal"));

        gridLayout->addWidget(setGoal, 0, 1, 1, 1);

        setPredefStart = new QPushButton(dockWidgetContents);
        setPredefStart->setObjectName(QString::fromUtf8("setPredefStart"));

        gridLayout->addWidget(setPredefStart, 0, 2, 1, 1);

        setPredefGoal = new QPushButton(dockWidgetContents);
        setPredefGoal->setObjectName(QString::fromUtf8("setPredefGoal"));

        gridLayout->addWidget(setPredefGoal, 0, 3, 1, 1);

        relocateObjects = new QPushButton(dockWidgetContents);
        relocateObjects->setObjectName(QString::fromUtf8("relocateObjects"));

        gridLayout->addWidget(relocateObjects, 1, 0, 1, 1);

        showStart = new QPushButton(dockWidgetContents);
        showStart->setObjectName(QString::fromUtf8("showStart"));

        gridLayout->addWidget(showStart, 1, 1, 1, 1);

        showGoal = new QPushButton(dockWidgetContents);
        showGoal->setObjectName(QString::fromUtf8("showGoal"));

        gridLayout->addWidget(showGoal, 1, 2, 1, 1);

        doPlan = new QPushButton(dockWidgetContents);
        doPlan->setObjectName(QString::fromUtf8("doPlan"));

        gridLayout->addWidget(doPlan, 1, 3, 1, 1);

        PlanningTabWidget->setWidget(dockWidgetContents);

        retranslateUi(PlanningTabWidget);

        QMetaObject::connectSlotsByName(PlanningTabWidget);
    } // setupUi

    void retranslateUi(QDockWidget *PlanningTabWidget)
    {
        PlanningTabWidget->setWindowTitle(QApplication::translate("PlanningTabWidget", "Planning Tab", 0, QApplication::UnicodeUTF8));
        setStart->setText(QApplication::translate("PlanningTabWidget", "Set Start", 0, QApplication::UnicodeUTF8));
        setGoal->setText(QApplication::translate("PlanningTabWidget", "Set Goal", 0, QApplication::UnicodeUTF8));
        setPredefStart->setText(QApplication::translate("PlanningTabWidget", "Set PredefStart", 0, QApplication::UnicodeUTF8));
        setPredefGoal->setText(QApplication::translate("PlanningTabWidget", "Set PredefGoal", 0, QApplication::UnicodeUTF8));
        relocateObjects->setText(QApplication::translate("PlanningTabWidget", "Relocate Objects", 0, QApplication::UnicodeUTF8));
        showStart->setText(QApplication::translate("PlanningTabWidget", "Show Start", 0, QApplication::UnicodeUTF8));
        showGoal->setText(QApplication::translate("PlanningTabWidget", "Show Goal", 0, QApplication::UnicodeUTF8));
        doPlan->setText(QApplication::translate("PlanningTabWidget", "Plan", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PlanningTabWidget: public Ui_PlanningTabWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PLANNINGTAB_H
