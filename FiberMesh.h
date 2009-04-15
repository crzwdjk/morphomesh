/*************************************************************
 *  Trevor O'Brien, trevor@cs.brown.edu
 *
 *  March 20, 2009
 *  FiberMesh.h
 *
 ************************************************************/

#ifndef FIBERMESH_H
#define FIBERMESH_H

#include <milton.h>
#include "ControlCurve.h"
#include "LinearSolver.h"
#include "FiberMesh.h"
#include "MeshUtils.h"
#include "MeshSkeleton.h"

class FiberMesh {

 public:
  
  FiberMesh() {
    m_At = SparseMatrix(0,0);
  }
  
  FiberMesh(ControlCurve curve, Vector2 screenSize, Camera *camera,
	    Vector3 color = Vector3(1.0f, 0.0f, 0.0f));

  FiberMesh(Mesh *mesh,
	    Vector3 color = Vector3(0.0f, 0.0f, 1.0));

  virtual ~FiberMesh() {
    delete m_camera;
    m_camera = NULL;

    delete m_3Dmesh;
    m_3Dmesh = NULL;
  }

  bool Create3DMesh(const std::vector<MeshTriangle> &faces);
  void RemoveBadTriangles();
  void ClearMeshData();

  void InitiallyInflate();
  
  bool OptimizeSurface();

  std::vector<float> GetCurrentCurvatures();
  std::vector<float> GetAverageEdgeLengths();

  void SetCamera(Camera *cam) {
    m_camera = cam;
  }

  void SetScreenSize(Vector2 screenSize) {
    m_screenSize = screenSize;
  }

  void MoveMesh(const Vector3 newCenter);

  bool GetTargetCurvatures(double *x);
  bool GetTargetEdges(double *x);
  bool GetTargetVerts(Vector3List verts, const double *curves,
                      const double *edges);

  Vector3List GetDeltas(const double *c);
  Vector3List GetEtas(const double *e);

  bool DeformCurve(Vector2 mousePos, unsigned int vertID, 
                   bool done = false);

  std::vector<unsigned int> GetBoundaryAndNeighbs(unsigned &numPts);

  Vector3List Get3DVerts() {
    return m_3Dverts;
  }

  unsigned int GetNum2D() {
    return m_boundary.GetUniformBoundary().size();
  }

  Mesh *GetMesh() {
    return m_3Dmesh;
  }

  void ComputeCentroid();
  void  ComputeCentroidPoint();

  Vector3 GetCentroid() {
    return m_centroid;
  }

  Point3 GetCentroidPoint() {
    return m_centroidPt;
  }

  void ProjectVertsToWorldSpace(const Vector2List &verts);

  Vector3 Color() {
    return m_color;
  }

  bool NormalizeMesh();

  bool ContractMesh();

  float GetAverageFaceAreaOfMesh();
  float GetAverageOneRingArea();

  std::vector<float> GetOneRingAreas();

 private:  

  Vector3List     m_3Dverts;
  MeshData        m_3DmeshData;
  Mesh            *m_3Dmesh;
  
  ControlCurve    m_boundary;

  // Fibermesh solving stuff.
  // -----------------------
  LinearSolver    m_curveSolver;
  LinearSolver    m_edgeSolver;
  LinearSolver    m_vertSolver;
  LinearSolver    m_deformSolver;

  SparseMatrix    m_At;
  SparseMatrix    m_Vt;
  SparseMatrix    m_Dt;
  SparseMatrix    m_Et;
  
  bool            m_cSolveInit;
  bool            m_eSolveInit;
  bool            m_vSolveInit;
  bool            m_dSolveInit;
  // ------------------------

  // Mesh contraction solving stuff.
  // ------------------------
  LinearSolver    m_contractSolver;
  SparseMatrix    m_contractT;

  unsigned int    m_contractIters;

  std::vector<float>  m_contractWL;
  std::vector<float>  m_contractWH;
  float               m_contractSL;

  std::vector<float>  m_origRingAreas;
  float               m_avgFaceArea;
  // ------------------------


  unsigned        m_handleRow;

  std::vector<unsigned int>  m_nonDeformers;
  Vector2         m_screenSize;
  Camera          *m_camera;

  bool            m_meshCreated;

  Vector3         m_color;
  Vector3         m_centroid;
  Point3          m_centroidPt;

  float           m_maxMag;

};

#endif
