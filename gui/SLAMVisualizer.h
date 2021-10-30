#ifndef SLAMVISUALIZER_H
#define SLAMVISUALIZER_H

#include <QWidget>
#include <qglviewer.h>

#include "GSLAM/core/GSLAM.h"

namespace GSLAM {

class SLAMVisualizerImpl;
class SLAMVisualizer : public QGLViewer, public GObjectHandle
{
public:
    SLAMVisualizer(QWidget *parent);

    void setSLAM(GSLAM::SLAM* s);
    GSLAM::SLAM* slam();

    void releaseSLAM();
    virtual void draw();
    virtual void handle(const SPtr<GObject>& obj);

protected:
    SPtr<SLAMVisualizerImpl>   impl;
};

} // end of namespace GSLAM

#endif // end of SLAMVISUALIZER_H
