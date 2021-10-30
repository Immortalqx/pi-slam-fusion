#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <memory>

#include <QMainWindow>
#include <QAction>

#include "GSLAM/core/GSLAM.h"
#include "GSLAM/core/Dataset.h"
#include "FrameVisualizer.h"
#include "SLAMVisualizer.h"


namespace GSLAM {

class MainWindowData;
class MainWindow: public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    virtual ~MainWindow(){slotStop();}

    virtual int setupLayout(void);

    void call(QString cmd);

    bool addSLAM(GSLAM::SLAM* s);

    FrameVisualizer* getFrameVisualizer(void) { return frameVis; }
    SLAMVisualizer*  getSLAMVisualizer(void) { return slamVis; }
    DatasetPtr       getDataset(){return dataset;}

signals:
    void call_signal(QString cmd);
    void signalStop();

public slots:
    void call_slot(QString cmd);
    void slotShowMessage(QString str,int msgType=0);
    bool slotOpen();
    bool slotOpen(QString file);
    bool slotStart();
    bool slotPause();
    bool slotStop();
    bool slotStartDataset(QString dataset);

protected:
    void keyPressEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void closeEvent(QCloseEvent *event);
    void timerEvent(QTimerEvent *event);

protected:
    DatasetPtr              dataset;    // current dataset, load implementations
    FrameVisualizer         *frameVis;  // image viewer
    SLAMVisualizer          *slamVis;   // SLAM visualizer

    std::shared_ptr<MainWindowData> _d;
};


class SCommandAction : public QAction
{
    Q_OBJECT
public:
    SCommandAction(const QString& cmd,const QString& text="",QMenu* parent=NULL);

public slots:
    void triggerdSlot();

private:
    QString _cmd;
};


} // end of namespace GSLAM

#endif // MAINWINDOW_H
