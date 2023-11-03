/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QPushButton *pushButton_Position;
    QPushButton *pushButton_Velocity;
    QCustomPlot *widget_RealTimeGraph;
    QPushButton *pushButton_Torque;
    QPushButton *pushButton_ExtWrench;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        pushButton_Position = new QPushButton(centralWidget);
        pushButton_Position->setObjectName(QString::fromUtf8("pushButton_Position"));

        gridLayout->addWidget(pushButton_Position, 3, 0, 1, 1);

        pushButton_Velocity = new QPushButton(centralWidget);
        pushButton_Velocity->setObjectName(QString::fromUtf8("pushButton_Velocity"));

        gridLayout->addWidget(pushButton_Velocity, 3, 1, 1, 1);

        widget_RealTimeGraph = new QCustomPlot(centralWidget);
        widget_RealTimeGraph->setObjectName(QString::fromUtf8("widget_RealTimeGraph"));

        gridLayout->addWidget(widget_RealTimeGraph, 0, 0, 1, 2);

        pushButton_Torque = new QPushButton(centralWidget);
        pushButton_Torque->setObjectName(QString::fromUtf8("pushButton_Torque"));

        gridLayout->addWidget(pushButton_Torque, 4, 0, 1, 1);

        pushButton_ExtWrench = new QPushButton(centralWidget);
        pushButton_ExtWrench->setObjectName(QString::fromUtf8("pushButton_ExtWrench"));

        gridLayout->addWidget(pushButton_ExtWrench, 4, 1, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1031, 22));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "RealTimeGraph", nullptr));
        pushButton_Position->setText(QApplication::translate("MainWindow", "Joint Position", nullptr));
        pushButton_Velocity->setText(QApplication::translate("MainWindow", "Joint Velocity", nullptr));
        pushButton_Torque->setText(QApplication::translate("MainWindow", "Joint Torque", nullptr));
        pushButton_ExtWrench->setText(QApplication::translate("MainWindow", "Apply External Wrench", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
