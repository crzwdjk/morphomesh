/*************************************************************
 *  Trevor O'Brien, trevor@cs.brown.edu
 *
 *  March 20, 2009
 *  FiberMesh.cpp
 *
 ************************************************************/
#include "FiberMesh.h"

FiberMesh::FiberMesh(ControlCurve curve, Vector2 screenSize, Camera *cam, Vector3 color) 
{
  m_boundary = curve;
  m_3Dmesh = NULL;

  m_screenSize = screenSize;
  m_camera = cam;

  // We haven't initialized any of our linear solvers to start.
  m_cSolveInit = false;
  m_eSolveInit = false;
  m_vSolveInit = false;
  m_dSolveInit = false;

  m_handleRow = 0;

  m_At = SparseMatrix(0,0);
  m_Et = SparseMatrix(0,0);
  m_Vt = SparseMatrix(0,0);
  m_Dt = SparseMatrix(0,0);

  m_color = color;

  // Project the points we've got into 3D and use our triangulation
  // to generate a pretty mesh we can look at.
  Vector2List all2Dverts = m_boundary.GetBoundsAndDups();
  ProjectVertsToWorldSpace(all2Dverts);
  
  std::vector<MeshTriangle> triangles = m_boundary.GetDuplicateTriangulation();
  m_meshCreated = Create3DMesh(triangles);
  
  if (!m_meshCreated) {
    cerr << "Uh, oh.  Something went wrong creating 3D mesh at Frontend::mouseReleaseEvent"
	 << endl;
  }
  
  // Ok.  Here we go.  Let's do the fancy stuff to optimize
  // our nice surface.
  if (!OptimizeSurface())
    cerr << "Problem optimizing surface.  Frontend::mouseReleaseEvent()" << endl;

  m_maxMag = 0.0f;

  ComputeCentroid();
  ComputeCentroidPoint();
  
  m_contractIters = 0;
  m_contractSL    = 6.0f;
  m_avgFaceArea   = 0;
}


FiberMesh::FiberMesh(Mesh *mesh, Vector3 color)
{
  m_screenSize = Vector2::zero();
  m_camera     = NULL;
  m_color      = color;

  m_3Dmesh     = mesh;

  ComputeCentroid();
  ComputeCentroidPoint();

  // cout << "The center of this mesh is: " << GetCentroid() << endl;

  MoveMesh(Vector3::zero());

  ComputeCentroid();
  ComputeCentroidPoint();
  // cout << "The new center of this mesh is: " << GetCentroid() << endl;

 //  if (NormalizeMesh())
//     cout << "Mesh normalized" << endl;
//   else
//     cerr << "Problem normalizing mesh." << endl;

  m_contractIters = 0;
  m_contractSL    = 2.0f;
  m_avgFaceArea   = 0;
}

void FiberMesh::ClearMeshData()
{
  m_3DmeshData.vertices.clear();
  m_3DmeshData.normals.clear();
  m_3DmeshData.uvs.clear();
  m_3DmeshData.triangles.clear();
  m_3DmeshData.fileName = "";
}


void FiberMesh::InitiallyInflate()
{
  if (m_3Dmesh == NULL) {
    cerr << "Can't inflate a mesh that doesn't exist! Frontend::InitiallyInflate()"
         << endl;

    return;
  }

  unsigned numNorms;
  m_3Dmesh->computeNormals();
  Normal *norms = m_3Dmesh->getNormals(numNorms);
  
  unsigned numVerts;
  Vertex *verts = m_3Dmesh->getVertices(numVerts);

  if (norms == NULL)
    cerr << "Umm, no norms here." << endl;

  Neighborhood *barrios;
  MeshUtils::getNeighbors(m_3Dmesh, &barrios);

  std::vector<float> edgeLengths = GetAverageEdgeLengths();
  float avgLen = 0.0f;
  
  for (unsigned int i = 0; i < edgeLengths.size(); i++)
    avgLen += edgeLengths[i];

  //cout << "Average edge length before: " << avgLen << endl;
  avgLen /= edgeLengths.size(); 

  //cout << "Average edge length: " << avgLen << endl;

  for (unsigned int i = m_boundary.GetUniformBoundary().size();
       i < numVerts; i++) {
    
    verts[i] += norms[i] * avgLen * 2.0f;
    if (!norms[i].isUnit())
      cerr << "Bad normal here! Frontend::InitiallyInflate()" << endl;
  }

  m_3Dmesh->computeNormals();
}


bool FiberMesh::OptimizeSurface()
{
  if (m_3Dmesh == NULL) {
    cerr << "Bad mesh! Frontend::OptimizeSurface()" << endl;
    return false;
  }

  cout << "Optimizing surface" << endl;

  int numIters = 25;

  for (int i = 0; i < numIters; i++) {
    
    float percentage = (float)(i)/numIters;

    cout << (int)(percentage * 100) << "% complete";
    unsigned int numVerts = m_3Dmesh->getNoVertices();
    
    cout << ".";
    double *c = new double[numVerts];
    if (!GetTargetCurvatures(c)) {
      cerr << "Couldn't get target curvatures" << endl;
      return false;
    } else {
      //cout << c[0] << endl;
    }
    
    cout << ".";

    double *e = new double[numVerts];
    if (!GetTargetEdges(e)) {
      cerr << "Couldn't get target edges" << endl;
      return false;
    } else {
      //cout << e[0] << endl;
    }
    
    cout << "." << endl;
    Vector3List newVerts;
    if (!GetTargetVerts(newVerts, c, e)) {
      cerr << "Couldn't get new vertex positions" << endl;
      return false;
    }

    delete [] c;
    c = NULL;
    delete [] e;
    e = NULL;
  }
  cout << "100% complete." << endl;

  return true;
}


bool FiberMesh::GetTargetCurvatures(double *x)
{

  if (m_3Dmesh == NULL) {
    cerr << "Bad mesh.  Frontend::GetTargetCurvatures()" << endl;
    return false;
  }

  unsigned numVerts = m_3Dmesh->getNoVertices();

  if (numVerts <= 0) {
    cerr << "No vertices to work with.  Frontend::GetTargetCurvatures()" << endl;
    return false;
  }

  SparseMatrix b(2 * numVerts, 1);

  // Fill in b with curvatures we know from our boundary.
  // (Fill zero everywhere else.)
  
  std::vector<float> curve;
  // if (m_cSolveInit == false) {
  //  curve = m_boundary.Curvatures();
  //} else {
  //cout << "Getting curr curve" << endl;
  curve = GetCurrentCurvatures();
  //cout << "Got curr curve" << endl;
  //}

  unsigned numBounds = GetNum2D();

  for (unsigned int i = 0; i < curve.size(); i++) {
    if (isnan(curve[i])) {
      b.setValue(i + numVerts, 0, 0);
    } else {
      if (i < numBounds)
	b.setValue(i + numVerts, 0, curve[i]);
      else 
	b.setValue(i + numVerts, 0, 0.1f * curve[i]);
    }
    // cout << "Curve: " << curve[i] << endl;
  }

  if (m_cSolveInit == false) {
    SparseMatrix A(2 * numVerts, numVerts);
    
    // Fill in our matrix A.
    // Start by filling in the top half...
    
    Neighborhood *barrios;
    MeshUtils::getNeighbors(m_3Dmesh, &barrios);
    
    //   for (int i = 0; i < 2 * numVerts; i++) {
    //     for (int j = 0; j < numVerts; j++) {
    //       A.setValue(i, j, 1.0f);
    //     }
    //   }
    
    for (unsigned int i = 0; i < numVerts; i++) {
      // For each vertex, find its neighbors.
      
      unsigned numNeighbs = barrios[i].noVertices;
      
      double ngbrValue = (double)(-1.0f/numNeighbs);
      
      for (unsigned int j = 0; j < numNeighbs; j++) {
        
        //  // cout << "Vert: " << barrios[i].vertices[j]<< " out of " << numVerts << endl;
        if ( barrios[i].vertices[j] > numVerts)
          cerr << "Bad matrix indexing" << endl;
        else
          A.setValue(i, barrios[i].vertices[j], ngbrValue);
      }
      
      if ( i > numVerts)
        cerr << "Bad matrix indexing" << endl;
      else
        A.setValue(i, i, 1.0f);
    }
    
    // And now fill the bottom half.
    
    // Use this weighting value for interior points, as suggested by Andrew
    // Nealan.  (see e-mail Travis fwded)
    float nonBoundWeight = 0.1f;
    
    unsigned numBounds = m_boundary.GetUniformBoundary().size();
    
    for (unsigned int i = 0; i < numBounds; i++) {
      if ( i + numVerts > 2 * numVerts)
        cerr << "Bad matrix indexing" << endl;
      else
      A.setValue(i + numVerts, i, 1.0f);
    }

    for (unsigned int i = numBounds; i < numVerts; i++) {
      if ( i + numVerts > 2 * numVerts)
        cerr << "Bad matrix indexing" << endl;
      else
        A.setValue(i + numVerts, i, nonBoundWeight);
    }
    
    m_At = A.getTranspose();

    SparseMatrix AtA = m_At * A;
    m_curveSolver.setA(AtA);
  }

  SparseMatrix AtB = m_At * b;

  //cout << "aTa: " << AtA.getM() << " by " << AtA.getN() << endl;
  //cout << "aTb: " << AtB.getM() << " by " << AtB.getN() << endl;

  double B[numVerts];

  for (unsigned int i = 0; i < numVerts; i++) {
    B[i] = AtB.getValue(i, 0);
  }

  if (!m_curveSolver.solve(B, x)) {
    cerr << "Couldn't solve for target curvatures. Frontend::ComputeTargetCurvatures()"
	 << endl;
    return false;
  }

  m_cSolveInit = true;

  return true;
}


bool FiberMesh::GetTargetEdges(double *x)
{
  // Solve equation 8 from the FiberMesh paper.
  // Very similar to what we have in equation 6, except here we are aiming
  // for target edge lengths.

  std::vector<float> avgEdges = GetAverageEdgeLengths();
  unsigned int numVerts = m_3Dmesh->getNoVertices();

  SparseMatrix e(2 * numVerts, 1);
  
  // Fill in e with edge lengths we know.
  for (unsigned int i = 0; i < avgEdges.size(); i++) {
    e.setValue(i + numVerts, 0, avgEdges[i]);
  }

  if (m_eSolveInit == false) {
    SparseMatrix A(2 * numVerts, numVerts);
    
    // Fill in our matrix A.
    // Start by filling in the top half...
    
    Neighborhood *barrios;
    MeshUtils::getNeighbors(m_3Dmesh, &barrios);
    
    //   for (int i = 0; i < 2 * numVerts; i++) {
    //     for (int j = 0; j < numVerts; j++) {
    //       A.setValue(i, j, 1.0f);
    //     }
    //   }
    
    for (unsigned int i = 0; i < numVerts; i++) {
      // For each vertex, find its neighbors.
      
      unsigned numNeighbs = barrios[i].noVertices;
      
      double ngbrValue = (double)(-1.0f/numNeighbs);
      
      for (unsigned int j = 0; j < numNeighbs; j++) {
	
        //  // cout << "Vert: " << barrios[i].vertices[j]<< " out of " << numVerts << endl;
        if ( barrios[i].vertices[j] > numVerts)
          cerr << "Bad matrix indexing" << endl;
        else
          A.setValue(i, barrios[i].vertices[j], ngbrValue);
      }
      
      if ( i > numVerts)
        cerr << "Bad matrix indexing" << endl;
      else
        A.setValue(i, i, 1.0f);
    }
    
    // And now fill the bottom half.
    
    // Use this weighting value for interior points, as suggested by Andrew
    // Nealan.  (see e-mail Travis fwded)
    //    float nonBoundWeight = 0.1f;
    
    unsigned numBounds = m_boundary.GetUniformBoundary().size();
    
    for (unsigned int i = 0; i < numVerts; i++) {
      if (i < numBounds) 
	A.setValue(i + numVerts, i, 1.0f);
      else
	A.setValue(i + numVerts, i, 0.1f);
    }
    
    //     for (unsigned int i = numBounds; i < numVerts; i++) {
    //       if ( i + numVerts > 2 * numVerts)
    //         cerr << "Bad matrix indexing" << endl;
    //       else
    //         A.setValue(i + numVerts, i, nonBoundWeight);
    //     }
    
    m_Et = A.getTranspose();
    
    SparseMatrix AtA = m_Et * A;
    m_edgeSolver.setA(AtA);

  }

  SparseMatrix AtB = m_Et * e;

  double B[numVerts];
  
  for (unsigned int i = 0; i < numVerts; i++) {
    B[i] = AtB.getValue(i, 0);
  }
  
  if (!m_edgeSolver.solve(B, x)) {
    cerr << "Couldn't solve for target edge lengths. Frontend::ComputeTargetEdges()"
	 << endl;
    return false;
  }
  
  m_eSolveInit = true;
  
  return true;
}


std::vector<float> FiberMesh::GetCurrentCurvatures()
{
  std::vector<float> currCurves;

  unsigned numVerts;
  Vertex *verts = m_3Dmesh->getVertices(numVerts);

  unsigned numNorms;
  Normal *norms = m_3Dmesh->getNormals(numNorms);

  Neighborhood *barrios;
  MeshUtils::getNeighbors(m_3Dmesh, &barrios);

  // First loop through, and for each vertex, sum up its 
  // neighbors.
  for (unsigned int i = 0; i < numVerts; i++) {
    Vertex neighbs(0.0f, 0.0f, 0.0f);
    Vertex neighNorms(0.0f, 0.0f, 0.0f);

    for (unsigned int j = 0; j < barrios[i].noVertices; j++) {
      neighbs += verts[barrios[i].vertices[j]];
      neighNorms += norms[barrios[i].vertices[j]];
    }

    // Divide by numNeighbors to get average of neighbors.
    if (barrios[i].noVertices > 0) {
      neighbs /= (double)barrios[i].noVertices;
      neighNorms /= (double)barrios[i].noVertices;
    }

    currCurves.push_back((verts[i] - neighbs).dot(neighNorms));
  }

  // Now we need to divide by one ring area to get correct curvature
  // magnitude.

  unsigned numTris;
  MeshTriangle *tris = m_3Dmesh->getTriangles(numTris);

  for (unsigned int i = 0; i < numVerts; i++) {
    float area = 0.0f;

    for (unsigned int j = 0; j < barrios[i].noTriangles; j++) {
      area += tris[barrios[i].triangles[j]].getSurfaceArea();
    }

    if (area > 0)
      currCurves[i] /= area;
  } 

  return currCurves;
}


std::vector<float> FiberMesh::GetAverageEdgeLengths()
{
  std::vector<float> edgeLengths;

  // Get the average edge lengths
  Neighborhood *barrios;
  MeshUtils::getNeighbors(m_3Dmesh, &barrios);

  unsigned int numVerts;
  Vertex *verts = m_3Dmesh->getVertices(numVerts);

  unsigned int nanCount = 0;

  for (unsigned int i = 0; i < numVerts; i++) {

    float avg = 0.0f;

    for (unsigned int j = 0; j < barrios[i].noVertices; j++) {

      double thisEdge = verts[i].getDistance(verts[barrios[i].vertices[j]]);
      
      if (isnan(thisEdge))
	nanCount++;
      else
	avg += thisEdge;
    }

    if (barrios[i].noVertices == nanCount) {
      edgeLengths.push_back(0);
    } else {
      avg /= (barrios[i].noVertices - nanCount);
      edgeLengths.push_back(avg);
    }
  }

  return edgeLengths;
}


bool FiberMesh::GetTargetVerts(Vector3List verts, const double *curves,
                               const double *edges)
{
  
  unsigned numBandN;
  std::vector<unsigned int> boundAndNgb = GetBoundaryAndNeighbs(numBandN);
  
  unsigned int numBounds = m_boundary.GetUniformBoundary().size();
  
  unsigned numVerts;
  Vertex *vertices = m_3Dmesh->getVertices(numVerts);
  
  double etaWeight = 0.005f;
  
  // If we haven't been here before, construct the A matrix we'll need.
  if (m_vSolveInit == false) {
    m_vSolveInit = true;
    
    SparseMatrix A(numVerts + numBounds + numBandN, numVerts);
    
    // Fill in our matrix A.
    // Start by filling in the top portion...
    
    Neighborhood *barrios;
    MeshUtils::getNeighbors(m_3Dmesh, &barrios);
    
    for (unsigned int i = 0; i < numVerts; i++) {
      // For each vertex, find its neighbors.
      
      unsigned numNeighbs = barrios[i].noVertices;
      
      double ngbrValue = (double)(-1.0f/numNeighbs);
      
      for (unsigned int j = 0; j < numNeighbs; j++) {
        A.setValue(i, barrios[i].vertices[j], ngbrValue);
      }
      
      A.setValue(i, i, 1.0f);
    }

    // Fill in the bottom.
    for (unsigned int i = 0; i < numBounds; i++) {
      A.setValue(i + numVerts, i, 100.0f);
    }

    // Fill in the very bottom.
    for (unsigned int i = 0; i < numBounds; i++) {
      
      for (unsigned int j = 0; j < barrios[i].noVertices; j++) {
        A.setValue(i + j + numVerts + numBounds, i, etaWeight);
        A.setValue(i + j + numVerts + numBounds,
                   barrios[i].vertices[j], -etaWeight);
      }
    }
    
    m_Vt = A.getTranspose();
    
    SparseMatrix AtA = m_Vt * A;
    m_vertSolver.setA(AtA);
  }
  
  SparseMatrix b(numVerts + numBounds + numBandN, 3);

  // Fill in the first numVerts elements with delta values.
  Vector3List delta = GetDeltas(curves);
  for (unsigned int i = 0; i < numVerts; i++) {
    b.setValue(i, 0, delta[i][0]);
    b.setValue(i, 1, delta[i][1]);
    b.setValue(i, 2, delta[i][2]);
  }

  // Fill in the next numBounds elements with current boundary
  // positions.
  for (unsigned int i = 0; i < numBounds; i++) {
    b.setValue(numVerts + i, 0, 100 * vertices[i][0]);
    b.setValue(numVerts + i, 1, 100 * vertices[i][1]);
    b.setValue(numVerts + i, 2, 100 * vertices[i][2]);
  }
    
  // Fill in the last numBandN elements with eta values.
  Vector3List eta = GetEtas(edges);
  for (unsigned int i = 0; i < numBandN; i++) {
    b.setValue(numVerts + numBounds + i, 0, eta[i][0] * etaWeight);
    b.setValue(numVerts + numBounds + i, 1, eta[i][1] * etaWeight);
    b.setValue(numVerts + numBounds + i, 2, eta[i][2] * etaWeight);
  }

  SparseMatrix AtB = m_Vt * b;
  
  double x[numVerts];
  double y[numVerts];
  double z[numVerts];

  // Solve for x coordinate.
  double X[numVerts];
  
  for (unsigned int i = 0; i < numVerts; i++) {
    X[i] = AtB.getValue(i, 0);
  }
  
  if (!m_vertSolver.solve(X, x)) {
    cerr << "Couldn't solve for target x coords."
	 << endl;
    return false;
  }
  
  // Solve for y coordinate.
  double Y[numVerts];
  
  for (unsigned int i = 0; i < numVerts; i++) {
    Y[i] = AtB.getValue(i, 1);
  }
  
  if (!m_vertSolver.solve(Y, y)) {
    cerr << "Couldn't solve for target x coords."
	 << endl;
    return false;
  }
  
  // Solve for z coordinate.
  double Z[numVerts];
  
  for (unsigned int i = 0; i < numVerts; i++) {
    Z[i] = AtB.getValue(i, 2);
  }
  
  if (!m_vertSolver.solve(Z, z)) {
    cerr << "Couldn't solve for target x coords."
	 << endl;
    return false;
  }

  // Update mesh.
  for (unsigned int i = 0; i < numVerts; i++) {
    //cout  << "Old pos: " << vertices[i] << endl;
    //cout  << "New pos: " << x[i] << " " << y[i] << " " << z[i] << endl;
    vertices[i][0] = x[i];
    vertices[i][1] = y[i];
    vertices[i][2] = z[i];
  }

  m_3Dmesh->computeNormals();

  return true;
}


std::vector<unsigned int> FiberMesh::GetBoundaryAndNeighbs(unsigned &numPts)
{
  std::vector<unsigned int> boundAndNgb;
  numPts = 0;

  // Start by getting the boundary.
  unsigned numBounds = m_boundary.GetUniformBoundary().size();

  Neighborhood *barrios;
  MeshUtils::getNeighbors(m_3Dmesh, &barrios);

  for (unsigned int i = 0; i < numBounds; i++) {

    for (unsigned int j = 0; j < barrios[i].noVertices; j++) {
      boundAndNgb.push_back(barrios[i].vertices[j]);
    }
  }

  numPts = boundAndNgb.size();

  return boundAndNgb;
}

bool FiberMesh::Create3DMesh(const std::vector<MeshTriangle> &faces)
{
  if (m_3Dverts.size() <= 0)
    return false;

  if (faces.size() <= 0)
    return false;

  // Clear up any previous garbage stored in our MeshData structure.
  ClearMeshData();

  m_3DmeshData.triangles = faces;
  m_3DmeshData.vertices = m_3Dverts;

  // Remove any bad triangles that aren't fully contained within our original
  // boundary curve.
  RemoveBadTriangles();
  
  m_3Dmesh = new Mesh(m_3DmeshData);

  if (m_3Dmesh == NULL) {
    return false;
  }

  m_3Dmesh->init();

  // Pump our mesh up just a touch, so initial curvature values are ok.
  InitiallyInflate();

  m_3Dmesh->init(true);

  return true;
}


void FiberMesh::RemoveBadTriangles()
{
  //std::vector<MeshTriangle> triangles = m_3DmeshData.triangles;
  Vector2List positions = m_boundary.GetBoundsAndDups();
  
  std::vector<unsigned int> forRemoval;

  for (unsigned int i = 0; i < m_3DmeshData.triangles.size(); i++) {

    // Get 2D positions for each triangle's vertices

    Vector2 aPos = positions[m_3DmeshData.triangles[i].A];
    Vector2 bPos = positions[m_3DmeshData.triangles[i].B];
    Vector2 cPos = positions[m_3DmeshData.triangles[i].C];

    Vector2 center = (aPos + bPos + cPos) / 3;

    if (!MeshUtils::isPointInPolygon(center, m_boundary.GetUniformBoundary())) {
      m_3DmeshData.triangles.erase(m_3DmeshData.triangles.begin() + i);
      i--;
      //cout << "Removed a bad triangle" << endl;
    }
  }
}


void FiberMesh::ProjectVertsToWorldSpace(const Vector2List &verts)
{
  // Don't want to append to a non-empty list.  Clear this sucker out.
  m_3Dverts.clear();

  Vector2 screenPt(Vector2::zero());
  Vector3 worldPt(Vector3::zero());

  for (unsigned int i = 0; i < verts.size(); i++) {
    
    screenPt = Vector2(verts[i][0]/m_screenSize[0], verts[i][1]/m_screenSize[1]);
    worldPt = MeshUtils::getWorldVertex(m_camera, screenPt);
    
    m_3Dverts.push_back(worldPt);
  }
  
}


void FiberMesh::ComputeCentroid() {
  unsigned numVerts;
  Vertex *verts = m_3Dmesh->getVertices(numVerts);
  
  Vector3 centerRot(Vector3::zero());

  m_maxMag = 0.0f;
  
  for (unsigned int i = 0; i < numVerts; i++) {
    centerRot += verts[i];

    if (verts[i].getMagnitude() > m_maxMag)
      m_maxMag = verts[i].getMagnitude();
  }
  
  centerRot /= numVerts;

  if (isnan(centerRot[0]) || isnan(centerRot[1]) || isnan(centerRot[1])) {
    m_centroid = Vector3::zero();
    return;
  }

  //cout << "Max magnitude is " << m_maxMag << endl;
  
  m_centroid = centerRot;
}


void FiberMesh::ComputeCentroidPoint() 
{
  Vector3 centerRot = GetCentroid();
  
  m_centroidPt = Point3(centerRot[0], centerRot[1], centerRot[2]);
}
  

Vector3List FiberMesh::GetDeltas(const double *c) 
{
  //cout << "Getting deltas" << endl;
  Vector3List deltas;
  
  unsigned numVerts = m_3Dmesh->getNoVertices();

  unsigned numTris;
  MeshTriangle *triangles = m_3Dmesh->getTriangles(numTris);

  unsigned numNorms;
  Normal *norms = m_3Dmesh->getNormals(numNorms);

  Neighborhood *barrios;
  MeshUtils::getNeighbors(m_3Dmesh, &barrios);

  for (unsigned int i = 0; i < numVerts; i++) {
    
    // Get average normal of neighbors

    Vector3 normal = norms[i];

    for (unsigned int j = 0; j < barrios[i].noVertices; j++) {
      normal += norms[barrios[i].vertices[j]];
    }

    if (barrios[i].noVertices > 0) {
      normal /= barrios[i].noVertices;
    }

    // Get one-ring triangle area
    float area = 0.0f;
    
    for (unsigned int j = 0; j < barrios[i].noTriangles; j++) {
      area += triangles[barrios[i].triangles[j]].getSurfaceArea();
    }

    area /= 3.0f;

    // cout << "Area: " << area << "C_i: " << c[i] << "Normal_i: " << normal << endl;
    deltas.push_back(area * c[i] * normal);
  }

  // cout << "Got deltas." << endl;
  return deltas;
}


Vector3List FiberMesh::GetEtas(const double *e)
{
  //cout << "Getting etas" << endl;
  Vector3List etas;

  // Compute eta values
  unsigned numVerts;
  Vertex *verts = m_3Dmesh->getVertices(numVerts);

  unsigned numBounds = m_boundary.GetUniformBoundary().size();

  std::vector<float> avgEdges = GetAverageEdgeLengths();

  Neighborhood *barrio;
  MeshUtils::getNeighbors(m_3Dmesh, &barrio);

  for (unsigned int i = 0; i < numBounds; i++) {
    
    for (unsigned int j = 0; j < barrio[i].noVertices; j++) {
      float edge = 0.5f * (avgEdges[i] - avgEdges[barrio[i].vertices[j]]);
      
      Vector3 vec = verts[i] - verts[barrio[i].vertices[j]];
      vec.normalize();

      etas.push_back(edge * vec);
    }
  }

  //cout << "Got etas" << endl;
  return etas;
}


bool FiberMesh::DeformCurve(Vector2 mousePos, unsigned int vertID, 
                            bool done)
{

  unsigned numIters = 1;
  if (done) {
    numIters = 15;
  }

  unsigned numVerts = m_3Dmesh->getNoVertices();

  // Determine where we should be moving this handle point.
  Vector2 screenPt(mousePos[0]/m_screenSize[0], mousePos[1]/m_screenSize[1]);
  Vector3 new3Dpt(MeshUtils::getWorldVertex(m_camera, screenPt));

  if (!m_dSolveInit) {
    // Determine which boundary points we need to keep
    // fixed.
    // For a guesstimate, let's use a quarter of the boundary
    // points in our deformation.
    unsigned int deformNbhd = 10;
    
    std::vector<unsigned int> deformers;
        
    deformers.push_back(vertID);

    for (unsigned int i = 1; i < deformNbhd; i++) {
      unsigned int ptFwd = vertID + i;
      unsigned int ptBwd = vertID - i;
      
      if (vertID + i >= GetNum2D()) {
        ptFwd = vertID + i - GetNum2D();
      }
      
      if (vertID - i < 0) {
        ptBwd = GetNum2D() + (vertID - i);
      }
      
      deformers.push_back(ptFwd);
      deformers.push_back(ptBwd);
    }
    
    for (unsigned int i = 0; i < GetNum2D(); i++) {
      bool found = false;
      for (unsigned int j = 0; j < deformers.size(); j++) {
        if (i == deformers[j])
          found = true;
      }
      
      if (!found)
        m_nonDeformers.push_back(i);
    }

    //cout << "Num non deformers " << m_nonDeformers.size() << endl;
    
    // Ok, now we have our fixed points.  Let's create
    // our matrices using the weight Andy Nealan suggested.
    SparseMatrix A(numVerts + m_nonDeformers.size() + 1, numVerts);
    
    // Fill in our matrix A.
    // Start by filling in the top half...
    Neighborhood *barrios;
    MeshUtils::getNeighbors(m_3Dmesh, &barrios);

    numVerts = m_3Dmesh->getNoVertices();

    for (unsigned int i = 0; i < numVerts; i++) {
      // For each vertex, find its neighbors.
      
      unsigned numNeighbs = barrios[i].noVertices;
      
      double ngbrValue = (double)(-1.0f/numNeighbs);
      
      for (unsigned int j = 0; j < numNeighbs; j++) {
        A.setValue(i, barrios[i].vertices[j], ngbrValue);
      }
      
      A.setValue(i, i, 1.0f);
    }
    
    // And now fill the bottom half.
    for (unsigned int i = 0; i < m_nonDeformers.size(); i++) {
      A.setValue(i + numVerts, m_nonDeformers[i], 100.0f);
    }

    A.setValue(m_nonDeformers.size() + numVerts, vertID, 100.0f);;
    
    m_Dt = A.getTranspose();
    
    SparseMatrix AtA = m_Dt * A;
    m_deformSolver.setA(AtA);
    
    m_dSolveInit = true;
  }

  // Do solving.
  for (unsigned int i = 0; i < numIters; i++) {
    
    Vertex *verts = m_3Dmesh->getVertices(numVerts);

    SparseMatrix b(numVerts + m_nonDeformers.size() + 1, 3);
    double *curves = new double[numVerts];
    if (!GetTargetCurvatures(curves)) {
      cerr << "GetTargetCurvatures failed" << endl;
    } else {
      Vector3List deltas = GetDeltas(curves);

      for (unsigned int j = 0; j < deltas.size(); j++) {
        b.setValue(j, 0, deltas[j][0]);
        b.setValue(j, 1, deltas[j][1]);
        b.setValue(j, 2, deltas[j][2]);
      }

      for (unsigned int j = 0; j < m_nonDeformers.size(); j++) {
	b.setValue(j + numVerts, 0, 100 * verts[m_nonDeformers[j]][0]);
	b.setValue(j + numVerts, 1, 100 * verts[m_nonDeformers[j]][1]);
	b.setValue(j + numVerts, 2, 100 * verts[m_nonDeformers[j]][2]); 
      }

      b.setValue(numVerts + m_nonDeformers.size(), 0, 100 * new3Dpt[0]);
      b.setValue(numVerts + m_nonDeformers.size(), 1, 100 * new3Dpt[1]);
      b.setValue(numVerts + m_nonDeformers.size(), 2, 100 * new3Dpt[2]);
      
      SparseMatrix AtB = m_Dt * b;
      
      double x[numVerts];
      double y[numVerts];
      double z[numVerts];
      
      // Solve for x coordinate.
      double X[numVerts];
      
      for (unsigned int i = 0; i < numVerts; i++) {
          X[i] = AtB.getValue(i, 0);
      }
      
      if (!m_deformSolver.solve(X, x)) {
        cerr << "Couldn't solve for target x coords."
             << endl;
        return false;
      }
      
      // Solve for y coordinate.
      double Y[numVerts];
      
      for (unsigned int i = 0; i < numVerts; i++) {
        Y[i] = AtB.getValue(i, 1);
      }
      
      if (!m_deformSolver.solve(Y, y)) {
        cerr << "Couldn't solve for target y coords."
             << endl;
        return false;
      }
      
      // Solve for z coordinate.
      double Z[numVerts];
      
      for (unsigned int i = 0; i < numVerts; i++) {
        Z[i] = AtB.getValue(i, 2);
      }
      
      if (!m_deformSolver.solve(Z, z)) {
        cerr << "Couldn't solve for target z coords."
             << endl;
        return false;
      }
      
      // Update mesh.
      for (unsigned int i = 0; i < numVerts; i++) {
        verts[i][0] = x[i];
        verts[i][1] = y[i];
        verts[i][2] = z[i];
      }

    //   // cout << "old pos: " << verts[m_handleRow][0] << " " 
//       //	   << verts[m_handleRow][1] << " " << verts[m_handleRow][2] << endl;
      
//       cout << "new pos: " << x[m_handleRow] << " "
//       	   << y[m_handleRow] << " " << z[m_handleRow] << endl;
//       cout << "intended pos: " << new3Dpt << endl;
      
      m_3Dmesh->computeNormals();
    }
  }
  
  // If that was the finishing touch, clear out our
  // initialized stuff.
  if (done) {
    m_dSolveInit = false;
    m_handleRow = 0;
    m_nonDeformers.clear();
  }

  return true;
}


void FiberMesh::MoveMesh(const Vector3 newCenter)
{
  Vector3 center = GetCentroid();
 
  unsigned int numVerts;
  Vertex *verts = m_3Dmesh->getVertices(numVerts);

  for (unsigned int i = 0; i < numVerts; i++)
    verts[i] -= (center + newCenter);
}


bool FiberMesh::NormalizeMesh()
{
  if (m_maxMag == 0)
    ComputeCentroid();

  if (m_maxMag == 0)
    return false;

  unsigned int numVerts;
  Vertex *verts = m_3Dmesh->getVertices(numVerts);

  for (unsigned int i = 0; i < numVerts; i++)
    verts[i] /= m_maxMag;

  m_3Dmesh->init(true);
  m_3Dmesh->computeNormals();

  return true;
}


bool FiberMesh::ContractMesh()
{
  unsigned numIters = 1;
  
  unsigned numVerts = m_3Dmesh->getNoVertices();

  cout << "Contracting " << numVerts << " vertices." << endl;

  if (m_contractIters == 0) {

    m_avgFaceArea = GetAverageFaceAreaOfMesh();
    //m_avgFaceArea = GetAverageOneRingArea();
    m_origRingAreas = GetOneRingAreas();

    cout << "average face area " << 1000 * sqrt(m_avgFaceArea) << endl;

    // Initialize weights for our system.
    for (unsigned int i = 0; i < numVerts; i++) {
      m_contractWL.push_back(1000 * sqrt(m_avgFaceArea));
      //m_contractWH.push_back(m_origRingAreas[i]);
      m_contractWH.push_back(1.0f);
    }  

    cout << "Laplacian weight" << m_contractWL[0] << endl;
  }

  // cout << "Initialized weights, creating system." << endl;
  // cout << "Here's our WL_0 value: " << m_contractWL[0] << endl;
  // cout << "original one rings: " << m_origRingAreas[0] << endl;
  // cout << "Ratio: " << m_contractWL[0]/m_contractWH[0] << endl;
  
  // Do solving.
  for (unsigned int i = 0; i < numIters; i++) {

    m_contractIters++;
    
    // Ok, now we have our fixed points.  Let's create
    // our matrices using the weight Andy Nealan suggested.
    SparseMatrix A(numVerts * 2, numVerts);
    
    // Fill in our matrix A.
    // Start by filling in the top half...
    Neighborhood *barrios;
    MeshUtils::getNeighbors(m_3Dmesh, &barrios);

    numVerts = m_3Dmesh->getNoVertices();

    for (unsigned int i = 0; i < numVerts; i++) {
      // For each vertex, find its neighbors.
      unsigned numNeighbs = barrios[i].noVertices;
      
      double ngbrValue = (double)(1.0f/numNeighbs);
      
      for (unsigned int j = 0; j < numNeighbs; j++) {
        A.setValue(i, barrios[i].vertices[j], m_contractWL[i] * ngbrValue);
      }
      
      A.setValue(i, i, m_contractWL[i] * -1.0f);
    }
    
    // And now fill the bottom half.
    for (unsigned int i = 0; i < numVerts; i++) {
      A.setValue(i + numVerts, i, m_contractWH[i]);
    }

    m_contractT = A.getTranspose();
    
    SparseMatrix AtA = m_contractT * A;
    m_contractSolver.setA(AtA);
    
    
    cout << "System created, solving now." << endl;
    
    // Ok, we've created our system, now fill in B
    // and solve for x accordingly.
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    Vertex *verts = m_3Dmesh->getVertices(numVerts);
    
    SparseMatrix b(numVerts * 2, 3);
    
    for (unsigned int j = 0; j < numVerts; j++) {
      b.setValue(j + numVerts, 0, m_contractWH[j] * verts[j][0]);
      b.setValue(j + numVerts, 1, m_contractWH[j] * verts[j][1]);
      b.setValue(j + numVerts, 2, m_contractWH[j] * verts[j][2]); 
    }
    
    SparseMatrix AtB = m_contractT * b;
    
    double x[numVerts];
    double y[numVerts];
    double z[numVerts];
    
    // Solve for x coordinate.
    double X[numVerts];
    
    for (unsigned int i = 0; i < numVerts; i++) {
      X[i] = AtB.getValue(i, 0);
    }
    
    if (!m_contractSolver.solve(X, x)) {
      cerr << "Couldn't solve for target x coords."
	   << endl;
      return false;
    }
    
    // Solve for y coordinate.
    double Y[numVerts];
    
    for (unsigned int i = 0; i < numVerts; i++) {
      Y[i] = AtB.getValue(i, 1);
    }
    
    if (!m_contractSolver.solve(Y, y)) {
      cerr << "Couldn't solve for target y coords."
	   << endl;
      return false;
    }
    
    // Solve for z coordinate.
    double Z[numVerts];
    
    for (unsigned int i = 0; i < numVerts; i++) {
      Z[i] = AtB.getValue(i, 2);
    }
    
    if (!m_contractSolver.solve(Z, z)) {
      cerr << "Couldn't solve for target z coords."
	   << endl;
      return false;
    }
    
    // Update mesh.
    for (unsigned int i = 0; i < numVerts; i++) {
      
      //  if (i > 100 && i % 100 == 0)
      // 	cout << "Old pos: " << verts[i] << endl;
      
      verts[i][0] = x[i];
      verts[i][1] = y[i];
      verts[i][2] = z[i];
      
      //  if (i > 100 && i % 100 == 0)
      // 	cout << "New pos: " << verts[i] << endl;
    }

    // Update weights.
    std::vector<float> currAreas = GetOneRingAreas();

    float avgContraction = 0;
    float maxContraction = 0;

    for (unsigned int i = 0; i < numVerts; i++) {
      // Increase collapsing speed by factor of SL.  (Given as 2.0 in paper.)
      m_contractSL = 1.5f;
      m_contractWL[i] = m_contractSL * m_contractWL[i];
      m_contractWH[i] = m_contractWH[i] * sqrt(m_origRingAreas[i]/currAreas[i]);

      avgContraction += m_contractWH[i];

      if (m_contractWH[i] > maxContraction)
	maxContraction = m_contractWH[i];
    }

    cout << "contraction value: " << m_contractWL[0] << endl; 
    cout << "average ratio: " << avgContraction/numVerts << endl;
    cout << "max ratio: " << maxContraction << endl;
  }

  cout << "Contracted." << endl;
    
  m_3Dmesh->computeNormals();

  return true;
}


float FiberMesh::GetAverageFaceAreaOfMesh()
{
  if (m_3Dmesh == NULL) {
    cerr << "Bad mesh.  FiberMesh::GetAverageFaceAreaOfMesh." << endl;
    return 0;
  }
  
  unsigned int numTris;
  MeshTriangle *tris = m_3Dmesh->getTriangles(numTris);

  float sumArea = 0.0f;

  for (unsigned int i = 0; i < numTris; i++) {
    sumArea += tris[i].getSurfaceArea();
  }

  return (sumArea / numTris);
}


float FiberMesh::GetAverageOneRingArea()
{
  std::vector<float> areas = GetOneRingAreas();

  float sum = 0.0f;

  for (unsigned int i = 0; i < areas.size(); i++)
    sum += areas[i];

  return (sum / areas.size());
}


std::vector<float> FiberMesh::GetOneRingAreas()
{
  std::vector<float> areas;

  if (m_3Dmesh == NULL) {
    cerr << "Bad mesh.  FiberMesh::GetOneRingArea." << endl;
    return areas;
  }

  unsigned int numTris;
  MeshTriangle *tris = m_3Dmesh->getTriangles(numTris);

  Neighborhood *barrio;
  MeshUtils::getNeighbors(m_3Dmesh, &barrio);

  unsigned numVerts = m_3Dmesh->getNoVertices();

  float sumArea = 0.0f;

  for (unsigned int j = 0; j < numVerts; j++) {
    for (unsigned int i = 0; i < barrio[j].noTriangles; i++) 
      sumArea += tris[barrio[j].triangles[i]].getSurfaceArea();

    areas.push_back(sumArea / barrio[j].noTriangles);
  }

  return areas;
}
