#ifndef CONTROLCURVE_H
#define CONTROLCURVE_H

#include "MeshUtils.h"
#include "Rect2D.h"

class ControlCurve {
public:

  ControlCurve() {}
  ControlCurve(Vector2List boundary);
  virtual ~ControlCurve() {}

  Vector2List GetRawBoundary() {
    return m_rawBoundary;
  }

  Vector2List GetUniformBoundary() {
    return m_uniformBoundary;
  }

  Vector2List GetInteriorPoints() {
    return m_interiorPoints;
  }

  void ComputeUniformBoundary();
  void ComputeInteriorPoints();

  Vector2 GetPointAtDistance(const float distance);

  bool PointInCurve(const Vector2 &pt);

  void Triangulate();
  
  std::vector<MeshTriangle> GetTriangulation() {
    return m_origTriangles;
  }

  void ComputeBoundingRect();

  Rect2D GetBoundingRect() {
    return m_boundingRect;
  }

  std::vector<float> Curvatures() {
    return m_boundCurvatures;
  }

  void ComputeBoundCurvatures();

  std::vector<MeshTriangle> GetDuplicateTriangulation() {
    return m_allTriangles;
  }

  void DuplicateAndTriangulate();

  Vector2List GetBoundsAndDups() {
    return m_boundsAndDups;
  }
    

private:

  Vector2List         m_rawBoundary;
  Vector2List         m_uniformBoundary;
  Vector2List         m_interiorPoints;
  Vector2List         m_boundsAndDups;
  
  std::vector<float>         m_boundCurvatures;
  std::vector<MeshTriangle>  m_origTriangles;
  std::vector<MeshTriangle>  m_allTriangles;

  Rect2D                     m_boundingRect;
};


#endif
