/*************************************************************
 *  Trevor O'Brien, trevor@cs.brown.edu
 *
 *  March 20, 2009
 *  ControlCurve.cpp
 *
 ************************************************************/

#include "ControlCurve.h"
#include "Rect2D.h"
#include <assert.h>

ControlCurve::ControlCurve(Vector2List boundary)
{
  m_rawBoundary = boundary;
  // Resample boundary.
  ComputeUniformBoundary();

  ComputeBoundingRect();
  
  cout << "Computing interior points." << endl;
  // Compute interioir points.
  ComputeInteriorPoints();
  
  cout << "Computing boundary curvatures" << endl;
  // Compute boundary curvatures.
  ComputeBoundCurvatures();
  
  cout << "Triangulating silhouette." << endl;
  // Duplicate interior verts and triangulate 
  DuplicateAndTriangulate();
  cout << "Control curve initialized." << endl;
}


void ControlCurve::ComputeInteriorPoints()
{
  float height = m_boundingRect.height;
  float width  = m_boundingRect.width;

  float samplingDistX = 0;
  float samplingDistY = 0;
  float divisor       = 15.0f;

  if (width > height) {
    samplingDistX = m_boundingRect.width/divisor;
    samplingDistY = samplingDistX/2 * sqrt(3);
  } else {
    samplingDistY = m_boundingRect.height/divisor;
    samplingDistX = 2 * samplingDistY/sqrt(3);
  }

  // Let's compute some nice interior points by generating a grid on top
  // of the bounding rect and perturbing them all in some randomized way.
  for (unsigned int i = 0; i * samplingDistX < m_boundingRect.width; i++) {
    for (unsigned int j = 0; j * samplingDistY < m_boundingRect.height; j++) {
      
      // Create gridded point.
      Vector2 pt = m_boundingRect.x0y0;
      
      
      pt += Vector2(i * samplingDistX, j * samplingDistY);

      if (j % 2 == 0) {  
        pt += Vector2(samplingDistX/2, 0);
      }
 
      // Kick it by some random amount for Delaunay triangulation to work.
      pt += Vector2(Random::sample(0, samplingDistX/10000),
		    Random::sample(0, samplingDistX/10000));
      
      // Add in our point if it's within our curve.
      if (PointInCurve(pt))
	m_interiorPoints.push_back(pt);      
    }
  }
}


bool ControlCurve::PointInCurve(const Vector2 &pt)
{
  return MeshUtils::isPointInPolygon(pt, m_uniformBoundary);
}


void ControlCurve::Triangulate()
{
  assert(m_boundsAndDups.size() > 0);

  if (!MeshUtils::triangulate(m_boundsAndDups, m_origTriangles))
    cerr << "Problem triangulating! ControlCurve::Triangulation" << endl;
}


Vector2 ControlCurve::GetPointAtDistance(const float distance)
{
  float curveDist = 0.0f;
  int ptIndex = -1;
  
  if (distance <= 0)
    return m_rawBoundary[0];

  // First figure out which little line segment the point we're looking
  // for lives on.
  while (curveDist < distance && 
         ptIndex < (int)m_rawBoundary.size() - 2) {
    
    ptIndex++;

    curveDist += (m_rawBoundary[ptIndex] - 
		  m_rawBoundary[ptIndex + 1]).getMagnitude();

  }

  if (curveDist < distance && ptIndex == (int)m_rawBoundary.size() - 2) {
    ptIndex = m_rawBoundary.size() - 1;
    curveDist += (m_rawBoundary[ptIndex] - 
                  m_rawBoundary[0]).getMagnitude();
  }

  // Now we've honed in on where we need to look for this point, let's
  // put a finer point on it.
  if (curveDist == distance) 
    return m_rawBoundary[ptIndex];

  if (curveDist < distance) {
    // You're looking for a distance that's longer than this curve :(
    printf("Bad ControlCurve::GetPointAtDistance() call %.2f %.2f\n",
           curveDist, distance);
    return Vector2::zero();
  }
  
  float remainingDist = distance - curveDist;


  if (ptIndex < (int)m_rawBoundary.size() - 1) {
  //  return 0.5 * (m_rawBoundary[ptIndex] + m_rawBoundary[ptIndex + 1]);
    Vector2 dir = (m_rawBoundary[ptIndex] - m_rawBoundary[ptIndex + 1]).getNormalized();
    return (m_rawBoundary[ptIndex] + remainingDist * dir);
  } else if (ptIndex == (int)m_rawBoundary.size() - 1) {
    //  return 0.5 * (m_rawBoundary[ptIndex] + m_rawBoundary[0]);
    Vector2 dir = (m_rawBoundary[ptIndex] - m_rawBoundary[0]).getNormalized();
    return (m_rawBoundary[ptIndex] + remainingDist * dir);
  } else {
    return Vector2::zero();
  }

}


void ControlCurve::ComputeUniformBoundary()
{
  float totalLength = 0;
  float deltaSample = 0;
  
  // First, determine total length of the curve we've got here...
  for (unsigned int i = 0; i < m_rawBoundary.size() - 1; i++) {
    totalLength += (m_rawBoundary[i] - m_rawBoundary[i + 1]).getMagnitude();
  }
  totalLength += (m_rawBoundary[m_rawBoundary.size() - 1] - 
                  m_rawBoundary[0]).getMagnitude();

  int divisor = m_rawBoundary.size();

  if (m_rawBoundary.size() > 100)
    divisor = 100;

  deltaSample = totalLength / divisor;

  for (unsigned int i = 0; i < divisor; i++) {
    Vector2 newPt = GetPointAtDistance(deltaSample * i);
    m_uniformBoundary.push_back(newPt);
  }
}


void ControlCurve::ComputeBoundingRect()
{
  // Find min, max of our boundary curve in the screen plane.
  float minX = 6000;
  float maxX = -1;
  float minY = 6000;
  float maxY = -1;

  for (unsigned int i = 0; i < m_uniformBoundary.size(); i++) {

    if (m_uniformBoundary[i][0] < minX)
      minX = m_uniformBoundary[i][0];

    if (m_uniformBoundary[i][0] > maxX)
      maxX = m_uniformBoundary[i][0];
    
    if (m_uniformBoundary[i][1] < minY)
      minY = m_uniformBoundary[i][1];
    
    if (m_uniformBoundary[i][1] > maxY)
      maxY = m_uniformBoundary[i][1];
  }

  //printf("Bounding rect -- x0: %.2f  y0: %.2f  width: %.2f  height:  %.2f\n",
  //	 minX, minY, maxX - minX, maxY - minY);
  
  m_boundingRect = Rect2D::xywh(Vector2(minX, minY), 
                                Vector2(maxX - minX, maxY - minY));
}


void ControlCurve::ComputeBoundCurvatures()
{
  assert(m_uniformBoundary.size() > 0);

  unsigned int lastElement = m_uniformBoundary.size() - 1;

  // Compute curvature at first point, which is a special case.
  float xDot = m_uniformBoundary[0][0] - m_uniformBoundary[lastElement][0];
  
  
  float yDot = m_uniformBoundary[0][1] - m_uniformBoundary[lastElement][1];
  
  float xDotDot = m_uniformBoundary[0][0] - 0.5f * (m_uniformBoundary[lastElement][0] + 
						    m_uniformBoundary[1][0]);
  
  float yDotDot = m_uniformBoundary[0][1] - 0.5f * (m_uniformBoundary[lastElement][1] + 
						    m_uniformBoundary[1][1]);
  
  float numerator   = xDot * yDotDot - yDot * xDotDot;
  float denominator = pow(xDot * xDot + yDot * yDot, 1.5f); 
  
  float curvature = 0;

  if (denominator == 0) {
    if (numerator >= 0)
      curvature = 0.1f;
    else
      curvature = -0.1f;
  } else {
    curvature = numerator/denominator;
  }

  m_boundCurvatures.push_back(curvature);

  for (unsigned int i = 1; i < m_uniformBoundary.size() - 1; i++) {

    // Simple curvature calculation we saw in class, and from the
    // FiberMesh warmup.

    xDot = (m_uniformBoundary[i][0] - m_uniformBoundary[i - 1][0]);
    
    yDot = (m_uniformBoundary[i][1] - m_uniformBoundary[i - 1][1]);

    xDotDot = m_uniformBoundary[i][0] - 0.5f * (m_uniformBoundary[i - 1][0] + 
					       m_uniformBoundary[i + 1][0]);
    
    yDotDot = m_uniformBoundary[i][1] - 0.5f * (m_uniformBoundary[i - 1][1] + 
					       m_uniformBoundary[i + 1][1]);

    numerator   = xDot * yDotDot - yDot * xDotDot;
    denominator = pow(xDot * xDot + yDot * yDot, 1.5f); 

    if (denominator == 0) {
      if (numerator >= 0)
	curvature = 0.1f;
      else
	curvature = -0.1f;
    } else {
      curvature   = numerator/denominator;
    }

    //cout << curvature << endl;
    m_boundCurvatures.push_back(curvature);
  }
  
  xDot = (m_uniformBoundary[lastElement][0] - m_uniformBoundary[lastElement - 1][0]);
  
  yDot = (m_uniformBoundary[lastElement][1] - m_uniformBoundary[lastElement - 1][1]);
  
  
  xDotDot = m_uniformBoundary[lastElement][0] - 
    0.5f * (m_uniformBoundary[lastElement - 1][0] + m_uniformBoundary[0][0]);
  
  
  yDotDot = m_uniformBoundary[lastElement][1] - 
    0.5f * (m_uniformBoundary[lastElement - 1][1] + m_uniformBoundary[0][1]);
  
  
  numerator   = xDot * yDotDot - yDot * xDotDot;
  denominator = pow(xDot * xDot + yDot * yDot, 1.5f); 

  if (denominator == 0) {
    if (numerator >= 0)
      curvature = 0.1f;
    else
      curvature = -0.1f;
  } else {
    curvature = numerator/denominator;
  }

  m_boundCurvatures.push_back(curvature);

//   // ------------------------------------------------------------
//   // We could potentially have that division by zero issue, so let's
//   // go through and replace all NaN entries with the maximum of what we've got.
//   //

//   float max = 0; 

//   for (unsigned int i = 0; i < m_boundCurvatures.size(); i++) {
//     if (!isnan(m_boundCurvatures[i]) && m_boundCurvatures[i] > max)
//       max = m_boundCurvatures[i];
//   }

//   for (unsigned int i = 0; i < m_boundCurvatures.size(); i++) {
//     if (isnan(m_boundCurvatures[i]))
//       m_boundCurvatures[i] = max;

//     cout << m_boundCurvatures[i] << endl;
//   }
}


void ControlCurve::DuplicateAndTriangulate() 
{
  m_boundsAndDups.clear();
  
  for (unsigned int i = 0; i < m_uniformBoundary.size(); i++) 
    m_boundsAndDups.push_back(m_uniformBoundary[i]);

  for (unsigned int i = 0; i < m_interiorPoints.size(); i++) 
    m_boundsAndDups.push_back(m_interiorPoints[i]);

  Triangulate();

  assert(m_origTriangles.size() > 0);

  cout << m_origTriangles.size() << endl;

  Vector2List currVerts = m_uniformBoundary;

  for (unsigned int i = 0; i < m_interiorPoints.size(); i++)
    currVerts.push_back(m_interiorPoints[i]);

  unsigned int numBounds = m_uniformBoundary.size();
  
  unsigned int numInterior = m_interiorPoints.size();

  // Sanity check.
  assert(numVerts == m_uniformBoundary.size() + m_interiorPoints.size());

  for (unsigned int i = 0; i < m_origTriangles.size(); i++)
    m_allTriangles.push_back(m_origTriangles[i]);

  for (unsigned int i = 0; i < m_origTriangles.size(); i++) {
    
    unsigned a = m_origTriangles[i].A;
    unsigned b = m_origTriangles[i].B;
    unsigned c = m_origTriangles[i].C;

    bool added = false;

    if (a >= numBounds) {
      a += numInterior;
      added = true;
    }
    
    if (b >= numBounds) {
      b += numInterior;
      added = true;
    }

    if (c >= numBounds) {
      c += numInterior;
      added = true;
    }
    
    // If we had a triangle that needed a duplicated point...
    if (added) {
      // Reverse the orientation of this new triangle and add it to our list.
      m_allTriangles.push_back(MeshTriangle(a, c, b));
    }
  }

  for (unsigned int i = 0; i < m_interiorPoints.size(); i++)
    m_boundsAndDups.push_back(m_interiorPoints[i]);
}
