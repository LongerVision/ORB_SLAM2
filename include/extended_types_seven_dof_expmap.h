#ifndef __EXTENDED_TYPES_SEVEN_DOF_EXPMAP_H__
#define __EXTENDED_TYPES_SEVEN_DOF_EXPMAP_H__

#include "g2o/types/sim3/types_seven_dof_expmap.h"

namespace g2o {
    
/**/
class EdgeInverseSim3ProjectXYZ : public  BaseBinaryEdge<2, Eigen::Vector2d,  VertexSBAPointXYZ, VertexSim3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeInverseSim3ProjectXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
      const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[1]);
      const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

      Eigen::Vector2d obs(_measurement);
      _error = obs-v1->cam_map(project(v1->estimate().inverse().map(v2->estimate())));
    }

   // virtual void linearizeOplus();

};

} // end namespace

#endif  // __EXTENDED_TYPES_SEVEN_DOF_EXPMAP_H__
