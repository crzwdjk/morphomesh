#ifndef FIBERMESH_UTILS_H
#define FIBERMESH_UTILS_H

#include "MeshUtils.h"

DECLARE_STL_TYPEDEF(std::vector<Vector3>, Vector3List);

class FiberMeshUtils : public MeshUtils {

 public:

  static Vector3 getWorldVertex(const Camera *camera, const Vector2 &filmpt,
				const float zOffset);

};

#endif
