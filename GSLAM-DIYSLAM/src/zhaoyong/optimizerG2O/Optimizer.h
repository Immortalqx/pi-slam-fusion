/*******************************************************************************

  Pilot Intelligence Library
    http://www.pilotintelligence.com/

  ----------------------------------------------------------------------------

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.

*******************************************************************************/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "GSLAM/core/Optimizer.h"

extern "C"{
    SPtr<GSLAM::Optimizer> createOptimizerInstance();
}

class OptimizerG2O: public GSLAM::Optimizer
{
public:
    // Update pose with 3D-2D corrospondences
    virtual bool optimizePnP(const std::vector<std::pair<GSLAM::Point3d,GSLAM::CameraAnchor> >& matches,
                             GSLAM::SE3& pose,GSLAM::KeyFrameEstimzationDOF dof=GSLAM::UPDATE_KF_SE3,double* information=NULL);

    virtual bool optimizePose(std::vector<std::pair<GSLAM::CameraAnchor,GSLAM::CameraAnchor> >& matches,
                              std::vector<GSLAM::IdepthEstimation>& firstIDepth,GSLAM::SE3&    relativePose,// T_{12}
                              GSLAM::KeyFrameEstimzationDOF dof=GSLAM::UPDATE_KF_SE3,double* information=NULL);

    virtual bool optimize(GSLAM::BundleGraph& graph);
    bool optimizeSE3Graph(GSLAM::BundleGraph& graph);
};
#endif // OPTIMIZER_H
