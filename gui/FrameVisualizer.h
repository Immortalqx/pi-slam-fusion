#ifndef FRAMEVISUALIZER_H
#define FRAMEVISUALIZER_H

#include <QWidget>
#include <QVBoxLayout>
#include <QSplitter>
#include <QTableWidget>

#include "GSLAM/core/GSLAM.h"
#include "GSLAM/core/Svar.h"

namespace GSLAM
{
class GImageWidget_local;

class InfomationViewer_local : public QTableWidget
{
    Q_OBJECT
public:
    InfomationViewer_local(QWidget* parent);
    QTableWidgetItem* setValue(int row,int col,QString val);
    QTableWidgetItem* setValue(int row,int col,double  val);
    void              update(const FramePtr& frame);
    std::map<QString,QString>   vars;
};

class FrameVisualizer_local : public QWidget,public GObjectHandle
{
    Q_OBJECT
public:
    FrameVisualizer_local(QWidget* parent=NULL):QWidget(parent){
        _splitter=new QSplitter(Qt::Vertical,this);
        _imageLayout=new QVBoxLayout(this);
        _imageLayout->addWidget(_splitter);
        _infos=new InfomationViewer_local(this);
        _splitter->addWidget(_infos);
        connect(this,SIGNAL(signalFrameUpdated()),this,SLOT(slotFrameUpdated()));
    }
    virtual ~FrameVisualizer_local(){}

    void setFrame(const FramePtr& frame){
        {
            GSLAM::WriteMutex lock(_mutex);
            _curFrame=frame;
        }
        if(!frame) return;
        emit signalFrameUpdated();
    }

    FramePtr curFrame(){return _curFrame;}

signals:
    void signalFrameUpdated();
public slots:
    void slotFrameUpdated();

protected:
    GSLAM::MutexRW                      _mutex;
    FramePtr                            _curFrame;
    std::vector<GImageWidget_local*>    _images;
    InfomationViewer_local*             _infos;
    QVBoxLayout*                        _imageLayout;
    QSplitter*                          _splitter;
};

} // end of namespace GSLAM

#endif // end of FRAMEVISUALIZER_H
