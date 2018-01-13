#ifndef GENERICPOPUP_H
#define GENERICPOPUP_H

#include <QWidget>
#include <QDialog>
#include <QTimer>
#include <iostream>
#include <vector>
#include <QListWidgetItem>

namespace Ui {
    class genericpopup;
}

class genericpopup : public QDialog
{
    Q_OBJECT

public:
    explicit genericpopup(QWidget *parent = 0);
    virtual ~genericpopup();
    void setPopupTitle(std::string text);
    void addCheckpoint(std::string messageText);
    void addPicture(QIcon pic);
    void addWarningMessage(std::string text);

public slots:
    void checkBoxChangeSignal(QListWidgetItem* clickedItem);


private slots:

    void on_doneButton_clicked();
    void on_retryButton_clicked();
    void on_abortButton_clicked();

signals:
    void send_doneButton_clicked();
    void send_retryButton_clicked();
    void send_abortButton_clicked();


private:
    Ui::genericpopup *ui;
//    typedef QPair<QCheckBox *, int> CheckBoxEntry;
    QList<QListWidgetItem*> checkBoxEntries;
    bool userWantsToShowWarningMsg;

};

#endif // GENERICPOPUP_H
