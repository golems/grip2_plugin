/********************************************************************************
** Form generated from reading UI file 'walktab.ui'
**
** Created: Tue Apr 8 10:51:40 2014
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
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_WalkTabWidget
{
public:
    QWidget *dockWidgetContents;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout;
    QLabel *label_2;
    QLabel *label;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *setStart;
    QSpacerItem *horizontalSpacer;
    QPushButton *setGoal;
    QSpacerItem *horizontalSpacer_3;

    void setupUi(QDockWidget *WalkTabWidget)
    {
        if (WalkTabWidget->objectName().isEmpty())
            WalkTabWidget->setObjectName(QString::fromUtf8("WalkTabWidget"));
        WalkTabWidget->resize(663, 182);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        horizontalLayout_2 = new QHBoxLayout(dockWidgetContents);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label_2 = new QLabel(dockWidgetContents);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout->addWidget(label_2);

        label = new QLabel(dockWidgetContents);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);

        setStart = new QPushButton(dockWidgetContents);
        setStart->setObjectName(QString::fromUtf8("setStart"));
        setStart->setMinimumSize(QSize(100, 50));

        horizontalLayout->addWidget(setStart);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        setGoal = new QPushButton(dockWidgetContents);
        setGoal->setObjectName(QString::fromUtf8("setGoal"));
        setGoal->setMinimumSize(QSize(100, 50));

        horizontalLayout->addWidget(setGoal);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_3);


        verticalLayout->addLayout(horizontalLayout);


        horizontalLayout_2->addLayout(verticalLayout);

        WalkTabWidget->setWidget(dockWidgetContents);

        retranslateUi(WalkTabWidget);

        QMetaObject::connectSlotsByName(WalkTabWidget);
    } // setupUi

    void retranslateUi(QDockWidget *WalkTabWidget)
    {
        WalkTabWidget->setWindowTitle(QApplication::translate("WalkTabWidget", "Walk Tab", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("WalkTabWidget", "Step1. Load the scene from data folder (04-world-Automatize.urdf)", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("WalkTabWidget", "Step2. Press \"Set Start\" and \"Set Goal\" and then press start button(green arrow) on the menu", 0, QApplication::UnicodeUTF8));
        setStart->setText(QApplication::translate("WalkTabWidget", "Set Start", 0, QApplication::UnicodeUTF8));
        setGoal->setText(QApplication::translate("WalkTabWidget", "Set Goal", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class WalkTabWidget: public Ui_WalkTabWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WALKTAB_H
