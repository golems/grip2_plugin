/********************************************************************************
** Form generated from reading UI file 'planningtab.ui'
**
** Created: Sat Apr 5 12:03:22 2014
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
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
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
    QVBoxLayout *verticalLayout2;
    QFrame *line_2;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_3;
    QLabel *label;
    QComboBox *nodeBox;
    QFrame *line;
    QVBoxLayout *verticalLayout_6;
    QLabel *label_2;
    QComboBox *dofBox;
    QFrame *line_4;
    QLabel *label_3;
    QComboBox *customBox;
    QSpacerItem *verticalSpacer;
    QFrame *line_5;
    QVBoxLayout *verticalLayout_2;
    QFrame *line_6;
    QLabel *label_4;
    QPushButton *pushButton_3;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QSpacerItem *verticalSpacer_2;
    QFrame *line_3;
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
        verticalLayout2 = new QVBoxLayout();
        verticalLayout2->setObjectName(QString::fromUtf8("verticalLayout2"));
        line_2 = new QFrame(dockWidgetContents);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout2->addWidget(line_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        label = new QLabel(dockWidgetContents);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_3->addWidget(label);

        nodeBox = new QComboBox(dockWidgetContents);
        nodeBox->setObjectName(QString::fromUtf8("nodeBox"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(nodeBox->sizePolicy().hasHeightForWidth());
        nodeBox->setSizePolicy(sizePolicy);

        verticalLayout_3->addWidget(nodeBox);


        horizontalLayout_3->addLayout(verticalLayout_3);

        line = new QFrame(dockWidgetContents);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);

        horizontalLayout_3->addWidget(line);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        label_2 = new QLabel(dockWidgetContents);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setAlignment(Qt::AlignCenter);

        verticalLayout_6->addWidget(label_2);

        dofBox = new QComboBox(dockWidgetContents);
        dofBox->setObjectName(QString::fromUtf8("dofBox"));
        sizePolicy.setHeightForWidth(dofBox->sizePolicy().hasHeightForWidth());
        dofBox->setSizePolicy(sizePolicy);

        verticalLayout_6->addWidget(dofBox);


        horizontalLayout_3->addLayout(verticalLayout_6);


        verticalLayout2->addLayout(horizontalLayout_3);

        line_4 = new QFrame(dockWidgetContents);
        line_4->setObjectName(QString::fromUtf8("line_4"));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);

        verticalLayout2->addWidget(line_4);

        label_3 = new QLabel(dockWidgetContents);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setAlignment(Qt::AlignCenter);

        verticalLayout2->addWidget(label_3);

        customBox = new QComboBox(dockWidgetContents);
        customBox->setObjectName(QString::fromUtf8("customBox"));

        verticalLayout2->addWidget(customBox);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout2->addItem(verticalSpacer);


        horizontalLayout->addLayout(verticalLayout2);

        line_5 = new QFrame(dockWidgetContents);
        line_5->setObjectName(QString::fromUtf8("line_5"));
        line_5->setFrameShape(QFrame::VLine);
        line_5->setFrameShadow(QFrame::Sunken);

        horizontalLayout->addWidget(line_5);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        line_6 = new QFrame(dockWidgetContents);
        line_6->setObjectName(QString::fromUtf8("line_6"));
        line_6->setFrameShape(QFrame::HLine);
        line_6->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_6);

        label_4 = new QLabel(dockWidgetContents);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(label_4);

        pushButton_3 = new QPushButton(dockWidgetContents);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));

        verticalLayout_2->addWidget(pushButton_3);

        pushButton = new QPushButton(dockWidgetContents);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        verticalLayout_2->addWidget(pushButton);

        pushButton_2 = new QPushButton(dockWidgetContents);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        verticalLayout_2->addWidget(pushButton_2);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_2);


        horizontalLayout->addLayout(verticalLayout_2);

        line_3 = new QFrame(dockWidgetContents);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);

        horizontalLayout->addWidget(line_3);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        customPlot = new QCustomPlot(dockWidgetContents);
        customPlot->setObjectName(QString::fromUtf8("customPlot"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(customPlot->sizePolicy().hasHeightForWidth());
        customPlot->setSizePolicy(sizePolicy1);

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
        label->setText(QApplication::translate("PlotTabWidget", "Robot Node", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("PlotTabWidget", "DOF", 0, QApplication::UnicodeUTF8));
        dofBox->clear();
        dofBox->insertItems(0, QStringList()
         << QApplication::translate("PlotTabWidget", "Joint", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("PlotTabWidget", "x", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("PlotTabWidget", "y", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("PlotTabWidget", "z", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("PlotTabWidget", "roll", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("PlotTabWidget", "pitch", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("PlotTabWidget", "yaw", 0, QApplication::UnicodeUTF8)
        );
        label_3->setText(QApplication::translate("PlotTabWidget", "Plugin Information", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("PlotTabWidget", "Actions", 0, QApplication::UnicodeUTF8));
        pushButton_3->setText(QApplication::translate("PlotTabWidget", "Insert", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("PlotTabWidget", "Remove", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QApplication::translate("PlotTabWidget", "Interact", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PlotTabWidget: public Ui_PlotTabWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PLANNINGTAB_H
