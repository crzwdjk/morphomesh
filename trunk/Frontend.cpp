/**<!-------------------------------------------------------------------->
   @file   Frontend.cpp
   @author Travis Fischer (fisch0920@gmail.com)
   @date   Spring 2009
   
   @brief
      Frontend for FiberMesh project
   
   @note this stencil comes with the ability to draw a curve and paint it 
      using Qt. It also contains basic, Maya-inspired camera interactions
      similar to the ones in Modeler if you took CS123. To try these camera 
      interactions out, use CTRL+left mouse to rotate, CTRL+middle mouse to 
      pan, and CTRL+right mouse to zoom in and out.
   <!-------------------------------------------------------------------->**/

#include "Frontend.h"
#include "ControlCurve.h"
#include "LinearSolver.h"
#include <milton.h>
#include "MeshSkeleton.h"

MeshSkeleton * skel = NULL;

using namespace std;

Frontend::Frontend(OpenGLCanvas *parent)
  : InteractionListener(parent)
{ 
  m_mouseDown = false;
  m_curveCreated = false;
  m_meshCreated = false;
  m_deforming = false;
  m_deformingID = 0;
  m_origMesh = NULL;
  m_contractFMesh = NULL;
}

Frontend::Frontend(OpenGLCanvas *parent, const std::string &fileName)
  : InteractionListener(parent)
{
  m_mouseDown = false;
  m_curveCreated = false;
  m_meshCreated = false;
  m_deforming = false;
  m_deformingID = 0;
  m_contractFMesh = NULL;

  m_origMesh = MeshLoader::load(fileName);

  if (m_origMesh == NULL) {
    cerr << "Loading mesh from " << fileName << " didn't go so well." << endl;
  } else {
    m_origMesh->init();
    m_origMesh->computeNormals();
    cout << "Creating fibermesh for contraction." << endl;
    m_contractFMesh = new FiberMesh(m_origMesh, Vector3(Random::sample(0, 1), 
							Random::sample(0, 1),
							Random::sample(0, 1)));
    cout << "Contraction mesh created." << endl;
  }
}


Frontend::~Frontend()
{ 
  //for (unsigned int i = 0; i < m_fiberMesh.size(); i++) {
  //  delete m_fiberMesh[i];
  //  m_fiberMesh[i] = NULL;
  //}
}

// MAGIC CONSTANT
const real_t drag_thresh = 3;

void Frontend::mousePressEvent  (InteractionInfo &info) {
   QMouseEvent *event = static_cast<QMouseEvent *>(info.event);
   
   if (event->modifiers() & Qt::ControlModifier) {
      _handleCameraMousePressEvent(event);
      return;
   }
   
   const Vector2 pt(event->x(), event->y());
   const Point2 pndc(pt[0]/getWidth(), pt[1]/getHeight(), 1);

   if (skel) {
       // find closest skel vertex. drag.
       int closest_point = -1;
       real_t min_distance = INFINITY;
       for (unsigned i = 0; i < skel->GetNodes().size(); i++) {
	   Point3 pnode = Point3(0, 0, 0, 1) + skel->GetNodes()[i];
	   real_t dist = getCamera()->getProjection(pnode).getDistance(pndc);
	   if (dist < min_distance) {
	       min_distance = dist;
	       closest_point = i;
	   }
       }
       if (min_distance < drag_thresh / getWidth()) {
	   m_deforming = true;
	   m_deformingVert = closest_point;
	   cerr << "drag " << closest_point << endl;
       }
   }

#if 0   
   Qt::MouseButtons buttons = event->buttons();
   if (buttons & Qt::LeftButton) {
     m_active.clear();
     m_active.push_back(pt);
     
     // TODO: interpret mouse click depending on whether you want 
     // to drag a previously drawn curve or draw a new curve
     
     m_parent->redraw();
     
     m_mouseDown = true;
   }

   if (buttons & Qt::RightButton) {
     // do curve dragging
     if (m_meshCreated && m_fiberMesh.size() > 0) {
       if (GetVertexClosestToMouse(pt, m_deformingID, m_deformingVert))
	 m_deforming = true;
     }
   }
#endif
}

void Frontend::mouseMoveEvent  (InteractionInfo &info) {
   QMouseEvent *event = static_cast<QMouseEvent *>(info.event);
   
   if (event->modifiers() & Qt::ControlModifier) {
      _handleCameraMouseMoveEvent(event);
      return;
   }
   
   const Vector2 pt(event->x(), event->y());
   if (m_deforming) {
       Ray r = getCamera()->getWorldRay(Point2(pt[0] / getWidth(), pt[1] / getHeight(), 1));
       Vector3 d = Point3() + skel->GetNodes()[m_deformingVert] - r.origin;
       skel->GetNodes()[m_deformingVert] = (r.origin + d.getMagnitude() * r.direction) - Point3();
       m_parent->redraw();
       return;
   }
   
#if 0
   if (m_deforming) {
     m_fiberMesh[m_deformingID]->DeformCurve(pt, m_deformingVert);
     m_parent->redraw();
   }

   if (m_mouseDown) {
     
     if (m_active.size() > 0 && m_active.back() != pt) {
       m_active.push_back(pt);
       m_parent->redraw();
     }
   }
#endif

}

void Frontend::mouseReleaseEvent(InteractionInfo &info) {
   const unsigned n = m_active.size();
   
   QMouseEvent *event = static_cast<QMouseEvent *>(info.event);   
   const Vector2 pt(event->x(), event->y());

   if (m_deforming) {
       skel->update();
       m_deforming = false;
       m_parent->redraw();
   }
#if 0
   if (m_deforming) {     
     m_fiberMesh[m_deformingID]->DeformCurve(pt, m_deformingVert, true);
     m_parent->redraw();
     m_deforming = false;
   }

   if (n > 0 && m_mouseDown) {

     m_mouseDown = false;

     // TODO: resample, triangulate, and inflate / optimize the completed curve
     // take note of MeshUtils::triangulate, and if you choose to use the 
     // Milton Mesh class, take note of the MeshData wrapper which can be used 
     // to construct a new Mesh via Mesh::Mesh(const MeshData&). Also be sure 
     // to call Mesh::init on your Mesh if you want to take advantage of all 
     // of its functionality
     
     // Create ControlCurve based on the points we've dragged out with the mouse.
     // The constructor of this class will take care of resampling the curve,
     // triangulating, and duplicating interior points.
     m_activeCurve = ControlCurve(m_active);
     m_curveCreated = true;

     FiberMesh *thisMesh = new FiberMesh(m_activeCurve, Vector2(getWidth(), getHeight()), 
					 getCamera(), Vector3(Random::sample(0,1),
							      Random::sample(0,1),
							      Random::sample(0,1)));

     if (thisMesh != NULL) {
       m_fiberMesh.push_back(thisMesh);
       m_meshCreated = true;
     }
     
     m_active.clear();
     m_parent->redraw();
   }
#endif     
}

void Frontend::keyPressEvent(InteractionInfo &info) {
   QKeyEvent *event = static_cast<QKeyEvent *>(info.event);
   
   if (event->modifiers() & Qt::ControlModifier)
      return; // handled by camera user interactions
   
   // example of handling key presses using Qt
   if (event->key() == Qt::Key_S) {
     if (m_contractFMesh != NULL) {
       // cout << "S down: Contracting mesh..." << endl;
       m_contractFMesh->ContractMesh();
       m_parent->redraw();
     }
   }
}

void Frontend::paintGL() {
   // TODO
   
   // Note if you have a Milton Mesh, you can preview it in OpenGL via 
   // Mesh::preview() which will do the whole glBegin/glEnd thing for you. Also 
   // note that Mesh::preview() caches an OpenGL display list for future 
   // calls to preview, so if you change the underlying geometry (say, via 
   // surface optimization), then make sure you call Mesh::init(true) and/or 
   // Mesh::setPreviewDirty() to force the Mesh to update its display list.

  if (m_mouseDown)
    return;
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  if (skel) {
      skel->draw();
  }

  if (m_meshCreated && m_fiberMesh.size() > 0) {
    for (unsigned int i = 0; i < m_fiberMesh.size(); i++) {
      if (m_fiberMesh[i]->GetMesh() != NULL) {

	Vector3 color = m_fiberMesh[i]->Color();
	Mesh *mesh = m_fiberMesh[i]->GetMesh();

	(*mesh->getMaterial())["kd"] = SpectralSampleSet(color[0], color[1],
							    color[2]);
	m_fiberMesh[i]->GetMesh()->preview();
	
	unsigned int numPoints = m_fiberMesh[i]->GetNum2D();

	unsigned int numVerts;
	Vertex *verts = m_fiberMesh[i]->GetMesh()->getVertices(numVerts);

	glColor3f(1.0f - color[0], 1.0f - color[1], 1.0f - color[2]);
	glLineWidth(2.0f);
	
	for (unsigned int i = 0; i < numPoints - 1; i++) {
	  glBegin(GL_LINES);
	  glVertex3f(verts[i][0], verts[i][1], verts[i][2]);
	  glVertex3f(verts[i + 1][0], verts[i + 1][1], verts[i + 1][2]);
	  glEnd();
	}

	//     glBegin(GL_LINES);
	//     glVertex3f(verts[numPoints - 1][0], verts[numPoints - 1][1], verts[numPoints - 1][2]);
	//     glVertex3f(verts[0][0], verts[0][1], verts[0][2]);
	//     glEnd();
      }
    }
  }

  if (m_contractFMesh != NULL && m_contractFMesh->GetMesh() != NULL) {
    Vector3 color = m_contractFMesh->Color();
    Mesh *mesh = m_contractFMesh->GetMesh();
    
    if (mesh != NULL) {
#if 0
       (*mesh->getMaterial())["kd"] = SpectralSampleSet(color[0], color[1],
							color[2]);
#endif
       glColor4d(color[0], color[1], color[2], 0.4);
       
       glBegin(GL_TRIANGLES);
       for (unsigned ti = 0; ti < mesh->getNoTriangles(); ti++) {
	   MeshTriangle t = mesh->getTriangles()[ti];
	   int v[3] = {t.A, t.B, t.C};
	   for (int vi = 0; vi < 3; vi++) {
	       glVertex3dv(mesh->getVertices()[v[vi]].data);
	   }
       }
       glEnd();

       //mesh->preview();
    }

    // Paint edges a different color.

    unsigned int numVerts;
    Vertex *verts = mesh->getVertices(numVerts);

    unsigned int numTriangles;
    MeshTriangle *tris = mesh->getTriangles(numTriangles);

    for (unsigned int j = 0; j < numTriangles; j++) {
      if (tris[j].getSurfaceArea() < 0.000001f) {
	// This is kind of 1D, draw edges distinctively.
	//glColor3f(1.0f - color[0], 1.0f - color[1], 1.0f - color[2]);
	glColor3f(1.0f, 0.0f, 0.0f);
	glLineWidth(2.0f);

	glBegin(GL_LINES);
	glVertex3f(verts[tris[j].A][0], verts[tris[j].A][1], verts[tris[j].A][2]);
	glVertex3f(verts[tris[j].B][0], verts[tris[j].B][1], verts[tris[j].B][2]);
	
	glVertex3f(verts[tris[j].C][0], verts[tris[j].C][1], verts[tris[j].C][2]);
	glVertex3f(verts[tris[j].B][0], verts[tris[j].B][1], verts[tris[j].B][2]);

	glVertex3f(verts[tris[j].A][0], verts[tris[j].A][1], verts[tris[j].A][2]);
	glVertex3f(verts[tris[j].C][0], verts[tris[j].C][1], verts[tris[j].C][2]);
	glEnd();
      }
    }
  } 
}
    

void Frontend::paint(QPainter *p) {
   QPen pen(Qt::red);
   pen.setWidth(3);
   
   // TODO: any non-OpenGL painting you wish to do
   // note: to project a point from world-space into NDC (normalized device 
   // coordinates, in [0,1]^2), use Camera::getProjection(Point3). to get the 
   // corresponding point's pixel coordinates, scale by the dimensions of the 
   // canvas; for example:
   // 
   // const unsigned width  = getWidth();
   // const unsigned height = getHeight();
   // Camera *camera = getCamera();
   // 
   // const Point3 &worldOrigin = Point3();
   // const Point2 &ndcPoint    = camera->getProjection(worldOrigin);
   // const unsigned x = ndcPoint[0] * width;  // horiz pixel coord
   // const unsigned y = ndcPoint[w] * height; // vert  pixel coord
   
   // draw currently active curve
   const unsigned n = m_active.size();
   
   if (n > 0) {
      // paint current, active curve
      pen.setColor(Qt::red);
      p->setPen(pen);
      
      if (m_mouseDown) {
	for(unsigned i = n - 1; i--;) {
	  const Vector2 &vA = m_active[i];
	  const Vector2 &vB = m_active[(i == n - 1 ? 0 : i + 1)];
	  
	  p->drawLine(QPointF(vA[0], vA[1]), QPointF(vB[0], vB[1]));
	}
	
      } 

      if (m_curveCreated) {
	
	Vector2List curvePts = m_activeCurve.GetUniformBoundary();
	
	pen.setColor(Qt::green);
	p->setPen(pen);
	
	//	for(unsigned i = n; i--;) {
	//  const Vector2 &vA = m_active[i];
	//  p->drawPoint(QPointF(vA[0], vA[1]));
	//}
	
// 	for (unsigned int i = 0; i < curvePts.size(); i++) {
//           const Vector2 &vA = curvePts[i];
// 	  //const Vector2 &vB = curvePts[(i == n - 1 ? 0 : i + 1)];
// 	  //p->drawPoint(QPointF(vA[0], vA[1]));
// 	  //p->drawLine(QPointF(vA[0], vA[1]), QPointF(vB[0], vB[1]));
// 	}

	pen.setColor(Qt::yellow);
	p->setPen(pen);

	Vector2List interiorPts = m_activeCurve.GetInteriorPoints();

	//cout << interiorPts.size() << endl;
	//for (unsigned int i = 0; i < interiorPts.size(); i++) {
	  //const Vector2 &vA = interiorPts[i];
	  //p->drawPoint(QPointF(vA[0], vA[1]));
	//}
	
      }
   }
}

void Frontend::init() {
   m_active.clear();
}

// CS123 Modeler-style mouse interactions
void Frontend::_handleCameraMousePressEvent(QMouseEvent *event) {
   const unsigned width  = m_parent->getWidth();
   const unsigned height = m_parent->getHeight();
   const Vector2 pt(event->x(), event->y());
   const Point2 filmPt(pt[0] / width, pt[1] / height);
   
   if (event->modifiers() & Qt::ControlModifier) {
      m_lastMousePos = m_mouseDownPos = Point2(event->x(), event->y());
      
      ThinLensCamera *camera = dynamic_cast<ThinLensCamera *>(m_parent->getCamera());
      ASSERT(camera);
      
      m_origEye  = camera->getEye();
      m_origUp   = camera->getUp();
      m_origLook = camera->getLook();
      m_origU    = camera->getU();
   }
}

// CS123 Modeler-style mouse interactions
void Frontend::_handleCameraMouseMoveEvent(QMouseEvent *event) {
   Qt::MouseButtons buttons = event->buttons();
   const Vector2 &diff = Point2(event->x(), event->y()) - m_lastMousePos;
   
   if (m_active.size() > 0)
      m_active.clear();
   
   ThinLensCamera *camera = dynamic_cast<ThinLensCamera *>(m_parent->getCamera());
   if (camera != NULL) {
      if (buttons & Qt::LeftButton) {
         // Camera trackball rotation
         // --------------------------------------------
         const Vector2 &diff = 
            Point2(event->x(), event->y()) - m_mouseDownPos;
         Point3 origin(0,0,0);
         
         // TODO (optional, but very useful): change 'origin' here to 
         // rotate around the intersection point on a mesh if the user clicked 
         // on a mesh or the world-space origin otherwise -- CTRL+left mouse 
         // will currently always rotate around the world-space origin
	 
	 // Ok, forget the ray tracing aspect, just have all of the rotations
	 // occur about the center of our mesh.
	 if (m_meshCreated && m_fiberMesh.size() > 0) {
	   origin = m_fiberMesh[m_fiberMesh.size() - 1]->GetCentroidPoint();
	 }
	 
	 // cout << "origin: " << origin << endl;
	 // cout << isnan(origin[0]) << endl;

         const real_t scaleFactor   = -0.015f;
         const Matrix4x4 &leftRight = getRotMat(origin, m_origUp, 
                                                scaleFactor * diff[0]);
         const Matrix4x4 &upDown    = getRotMat(origin, m_origU, 
                                                scaleFactor * diff[1]);
         
         // Calculate new camera parameters
         const Point3 &eyeNew   = upDown * leftRight * m_origEye;
         const Vector3 &upNew   = upDown * leftRight * m_origUp;
         const Vector3 &lookNew = upDown * leftRight * m_origLook;
         
         camera->setOrientation(eyeNew, lookNew, upNew);
      } else if (buttons & Qt::RightButton) {
         // Camera dolly (translation along look vector)
         // --------------------------------------------
         const real_t   amount = diff[0] / 200.0;
         const Vector3 &offset = camera->getLook() * amount;
         
         const Point3 &eyeNew  = camera->getEye() + offset;
         camera->setOrientation(eyeNew, camera->getLook(), camera->getUp());
      } else if (buttons & Qt::MidButton) {
         // Camera pan (translation along uv film plane)
         // --------------------------------------------
         const Point3 &eyeNew  = camera->getEye() + 
            (camera->getU() * (-diff[0] / 64))    + 
            (camera->getV() * (diff[1] / 64));
         
         camera->setOrientation(eyeNew, camera->getLook(), camera->getUp());
      }
      
      m_parent->redraw();
   } 
   
   m_lastMousePos = Point2(event->x(), event->y());
}



bool Frontend::GetVertexClosestToMouse(const Vector2 pt, unsigned int &meshIndex,
                                        unsigned int &vertIndex)
{
  Point2 screenPt(pt[0]/getWidth(), pt[1]/getHeight());

  meshIndex = 0;
  vertIndex = 0;

  double dist = getWidth() + getHeight();
  double thisDist = 0;

  for (unsigned int i = 0; i < m_fiberMesh.size(); i++) {
    
    unsigned int numVerts;
    Vertex *verts = m_fiberMesh[i]->GetMesh()->getVertices(numVerts);
    
    unsigned int numBounds = m_fiberMesh[i]->GetNum2D();

    for (unsigned int j = 0; j < numBounds; j++) {
      Point3 worldPt(verts[j][0], verts[j][1], verts[j][2]);
      Point2 projected(getCamera()->getProjection(worldPt));

      thisDist = screenPt.getDistance(projected);

      if (thisDist < dist) {
        dist = thisDist;
        meshIndex = i;
        vertIndex = j;
      }
    }
  }
  
  if (dist < 0.025f)
    return true;
  else
    return false;
}



