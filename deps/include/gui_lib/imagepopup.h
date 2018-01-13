#ifndef IMAGEPOPUP_H
#define IMAGEPOPUP_H

#include <QWidget>
#include <QDialog>
#include <QTimer>
#include <iostream>
#include <vector>
#include <QListWidgetItem>
#include <QDebug>

namespace Ui {
    class imagepopup;
}

class imagepopup : public QDialog
{
    Q_OBJECT

public:
    explicit imagepopup(QWidget *parent = 0);
    virtual ~imagepopup();
    void setPopupTitle(std::string text);
    void enlargeLiveImage();

public slots:
    void updateLiveImage(const QImage& img);

private slots:

    void on_doneButton_clicked();
    void on_retryButton_clicked();
    void on_abortButton_clicked();

    void on_rotateButton_clicked();

signals:
    void send_doneButton_clicked();
    void send_rotateButton_clicked();
    void send_retryButton_clicked();
    void send_abortButton_clicked();

private:
    Ui::imagepopup *ui;
    bool userWantsToShowWarningMsg;
};

#endif // IMAGEPOPUP_H
