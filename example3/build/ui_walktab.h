/********************************************************************************
** Form generated from reading UI file 'walktab.ui'
**
** Created: Wed Apr 2 16:11:43 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WALKTAB_H
#define UI_WALKTAB_H

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

class Ui_WalkTabWidget
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

    void setupUi(QDockWidget *WalkTabWidget)
    {
        if (WalkTabWidget->objectName().isEmpty())
            WalkTabWidget->setObjectName(QString::fromUtf8("WalkTabWidget"));
        WalkTabWidget->resize(663, 182);
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

        WalkTabWidget->setWidget(dockWidgetContents);

        retranslateUi(WalkTabWidget);

        QMetaObject::connectSlotsByName(WalkTabWidget);
    } // setupUi

    void retranslateUi(QDockWidget *WalkTabWidget)
    {
        WalkTabWidget->setWindowTitle(QApplication::translate("WalkTabWidget", "Walk Tab", 0, QApplication::UnicodeUTF8));
        setStart->setText(QApplication::translate("WalkTabWidget", "Set Start", 0, QApplication::UnicodeUTF8));
        setGoal->setText(QApplication::translate("WalkTabWidget", "Set Goal", 0, QApplication::UnicodeUTF8));
        setPredefStart->setText(QApplication::translate("WalkTabWidget", "Set PredefStart", 0, QApplication::UnicodeUTF8));
        setPredefGoal->setText(QApplication::translate("WalkTabWidget", "Set PredefGoal", 0, QApplication::UnicodeUTF8));
        relocateObjects->setText(QApplication::translate("WalkTabWidget", "Relocate Objects", 0, QApplication::UnicodeUTF8));
        showStart->setText(QApplication::translate("WalkTabWidget", "Show Start", 0, QApplication::UnicodeUTF8));
        showGoal->setText(QApplication::translate("WalkTabWidget", "Show Goal", 0, QApplication::UnicodeUTF8));
        doPlan->setText(QApplication::translate("WalkTabWidget", "Plan", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class WalkTabWidget: public Ui_WalkTabWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WALKTAB_H
