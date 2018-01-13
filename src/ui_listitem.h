/********************************************************************************
** Form generated from reading UI file 'listitem.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LISTITEM_H
#define UI_LISTITEM_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ListItem
{
public:
    QHBoxLayout *horizontalLayout;
    QGridLayout *gridLayout_2;
    QLabel *boxID_value;
    QLabel *boxID_icon;
    QGridLayout *gridLayout;
    QLabel *width_value;
    QLabel *length_value;
    QLabel *length_label;
    QSpacerItem *horizontalSpacer_2;
    QLabel *height_label;
    QLabel *width_label;
    QSpacerItem *horizontalSpacer;
    QLabel *height_value;
    QSpacerItem *horizontalSpacer_3;
    QFrame *line;
    QSpacerItem *horizontalSpacer_6;
    QFrame *line_2;
    QSpacerItem *horizontalSpacer_7;
    QVBoxLayout *verticalLayout_3;
    QLabel *tool_value;
    QLabel *tool_label;

    void setupUi(QWidget *ListItem)
    {
        if (ListItem->objectName().isEmpty())
            ListItem->setObjectName(QString::fromUtf8("ListItem"));
        ListItem->setEnabled(true);
        ListItem->resize(680, 120);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(ListItem->sizePolicy().hasHeightForWidth());
        ListItem->setSizePolicy(sizePolicy);
        ListItem->setMinimumSize(QSize(680, 120));
        ListItem->setMaximumSize(QSize(680, 132));
        ListItem->setStyleSheet(QString::fromUtf8("#ListItem {\n"
"	background-color: rgb(40, 40, 40);\n"
"}\n"
"\n"
"QLabel[class=\"textLabel\"] {\n"
"	color: rgb(138, 138, 138);\n"
"	font-family: AvenirNext LT Pro Regular;\n"
"	font-size: 14px;\n"
"}\n"
"\n"
"QLabel[class=\"textValue\"] {\n"
"	color: rgb(240, 240, 240);\n"
"	font-family: AvenirNext LT Pro Regular;\n"
"	font-size: 20px;\n"
"}\n"
"\n"
"#boxID_value {\n"
"	color: rgb(240, 240, 240);\n"
"	font-family: AvenirNext LT Pro Regular;\n"
"	font-size: 14px;\n"
"}\n"
"\n"
"QFrame {\n"
"	color: rgb(48, 48, 48);\n"
"}\n"
""));
        horizontalLayout = new QHBoxLayout(ListItem);
        horizontalLayout->setSpacing(0);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(20, 20, 20, 20);
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        boxID_value = new QLabel(ListItem);
        boxID_value->setObjectName(QString::fromUtf8("boxID_value"));
        boxID_value->setMinimumSize(QSize(100, 20));
        boxID_value->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(boxID_value, 1, 0, 1, 1);

        boxID_icon = new QLabel(ListItem);
        boxID_icon->setObjectName(QString::fromUtf8("boxID_icon"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(boxID_icon->sizePolicy().hasHeightForWidth());
        boxID_icon->setSizePolicy(sizePolicy1);
        boxID_icon->setMinimumSize(QSize(60, 64));
        boxID_icon->setPixmap(QPixmap(QString::fromUtf8(":/listitem/box_icon.png")));
        boxID_icon->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(boxID_icon, 0, 0, 1, 1);


        horizontalLayout->addLayout(gridLayout_2);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        width_value = new QLabel(ListItem);
        width_value->setObjectName(QString::fromUtf8("width_value"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Minimum);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(width_value->sizePolicy().hasHeightForWidth());
        width_value->setSizePolicy(sizePolicy2);
        width_value->setMinimumSize(QSize(0, 30));

        gridLayout->addWidget(width_value, 0, 3, 1, 1);

        length_value = new QLabel(ListItem);
        length_value->setObjectName(QString::fromUtf8("length_value"));
        sizePolicy1.setHeightForWidth(length_value->sizePolicy().hasHeightForWidth());
        length_value->setSizePolicy(sizePolicy1);
        length_value->setMinimumSize(QSize(0, 30));

        gridLayout->addWidget(length_value, 0, 1, 1, 1);

        length_label = new QLabel(ListItem);
        length_label->setObjectName(QString::fromUtf8("length_label"));
        sizePolicy1.setHeightForWidth(length_label->sizePolicy().hasHeightForWidth());
        length_label->setSizePolicy(sizePolicy1);
        length_label->setMinimumSize(QSize(80, 20));

        gridLayout->addWidget(length_label, 1, 1, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(30, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer_2, 0, 2, 1, 1);

        height_label = new QLabel(ListItem);
        height_label->setObjectName(QString::fromUtf8("height_label"));
        sizePolicy1.setHeightForWidth(height_label->sizePolicy().hasHeightForWidth());
        height_label->setSizePolicy(sizePolicy1);
        height_label->setMinimumSize(QSize(80, 20));

        gridLayout->addWidget(height_label, 1, 5, 1, 1);

        width_label = new QLabel(ListItem);
        width_label->setObjectName(QString::fromUtf8("width_label"));
        sizePolicy1.setHeightForWidth(width_label->sizePolicy().hasHeightForWidth());
        width_label->setSizePolicy(sizePolicy1);
        width_label->setMinimumSize(QSize(80, 20));

        gridLayout->addWidget(width_label, 1, 3, 1, 1);

        horizontalSpacer = new QSpacerItem(60, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 0, 0, 1, 1);

        height_value = new QLabel(ListItem);
        height_value->setObjectName(QString::fromUtf8("height_value"));
        QSizePolicy sizePolicy3(QSizePolicy::Expanding, QSizePolicy::Minimum);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(height_value->sizePolicy().hasHeightForWidth());
        height_value->setSizePolicy(sizePolicy3);
        height_value->setMinimumSize(QSize(0, 30));

        gridLayout->addWidget(height_value, 0, 5, 1, 1);

        horizontalSpacer_3 = new QSpacerItem(30, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer_3, 0, 4, 1, 1);


        horizontalLayout->addLayout(gridLayout);

        line = new QFrame(ListItem);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShadow(QFrame::Plain);
        line->setLineWidth(2);
        line->setFrameShape(QFrame::VLine);

        horizontalLayout->addWidget(line);

        horizontalSpacer_6 = new QSpacerItem(10, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_6);

        line_2 = new QFrame(ListItem);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShadow(QFrame::Plain);
        line_2->setLineWidth(2);
        line_2->setFrameShape(QFrame::VLine);

        horizontalLayout->addWidget(line_2);

        horizontalSpacer_7 = new QSpacerItem(20, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_7);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(0);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        tool_value = new QLabel(ListItem);
        tool_value->setObjectName(QString::fromUtf8("tool_value"));
        sizePolicy1.setHeightForWidth(tool_value->sizePolicy().hasHeightForWidth());
        tool_value->setSizePolicy(sizePolicy1);
        tool_value->setMinimumSize(QSize(40, 64));
        tool_value->setAlignment(Qt::AlignCenter);

        verticalLayout_3->addWidget(tool_value);

        tool_label = new QLabel(ListItem);
        tool_label->setObjectName(QString::fromUtf8("tool_label"));
        sizePolicy1.setHeightForWidth(tool_label->sizePolicy().hasHeightForWidth());
        tool_label->setSizePolicy(sizePolicy1);
        tool_label->setMinimumSize(QSize(80, 20));
        tool_label->setAlignment(Qt::AlignCenter);

        verticalLayout_3->addWidget(tool_label);


        horizontalLayout->addLayout(verticalLayout_3);


        retranslateUi(ListItem);

        QMetaObject::connectSlotsByName(ListItem);
    } // setupUi

    void retranslateUi(QWidget *ListItem)
    {
        ListItem->setWindowTitle(QApplication::translate("ListItem", "Form", 0, QApplication::UnicodeUTF8));
        boxID_value->setText(QApplication::translate("ListItem", "123456789", 0, QApplication::UnicodeUTF8));
        boxID_value->setProperty("class", QVariant(QApplication::translate("ListItem", "textLabel", 0, QApplication::UnicodeUTF8)));
        boxID_icon->setText(QString());
        boxID_icon->setProperty("class", QVariant(QApplication::translate("ListItem", "textValue", 0, QApplication::UnicodeUTF8)));
        width_value->setText(QApplication::translate("ListItem", "1.20", 0, QApplication::UnicodeUTF8));
        width_value->setProperty("class", QVariant(QApplication::translate("ListItem", "textValue", 0, QApplication::UnicodeUTF8)));
        length_value->setText(QApplication::translate("ListItem", "1.20", 0, QApplication::UnicodeUTF8));
        length_value->setProperty("class", QVariant(QApplication::translate("ListItem", "textValue", 0, QApplication::UnicodeUTF8)));
        length_label->setText(QApplication::translate("ListItem", "length (m)", 0, QApplication::UnicodeUTF8));
        length_label->setProperty("class", QVariant(QApplication::translate("ListItem", "textLabel", 0, QApplication::UnicodeUTF8)));
        height_label->setText(QApplication::translate("ListItem", "height (m)", 0, QApplication::UnicodeUTF8));
        height_label->setProperty("class", QVariant(QApplication::translate("ListItem", "textLabel", 0, QApplication::UnicodeUTF8)));
        width_label->setText(QApplication::translate("ListItem", "width (m)", 0, QApplication::UnicodeUTF8));
        width_label->setProperty("class", QVariant(QApplication::translate("ListItem", "textLabel", 0, QApplication::UnicodeUTF8)));
        height_value->setText(QApplication::translate("ListItem", "1.20 ", 0, QApplication::UnicodeUTF8));
        height_value->setProperty("class", QVariant(QApplication::translate("ListItem", "textValue", 0, QApplication::UnicodeUTF8)));
        tool_value->setText(QString());
        tool_value->setProperty("class", QVariant(QApplication::translate("ListItem", "textValue", 0, QApplication::UnicodeUTF8)));
        tool_label->setText(QApplication::translate("ListItem", "tool", 0, QApplication::UnicodeUTF8));
        tool_label->setProperty("class", QVariant(QApplication::translate("ListItem", "textLabel", 0, QApplication::UnicodeUTF8)));
    } // retranslateUi

};

namespace Ui {
    class ListItem: public Ui_ListItem {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LISTITEM_H
