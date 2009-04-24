/**<!-------------------------------------------------------------------->
   @file   Frontend.h
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

#ifndef FIBERMESH_FRONTEND_H_
#define FIBERMESH_FRONTEND_H_

#include <Visualization.h>
#include <OpenGLCanvas.h>
#include "MeshUtils.h"
#include "ControlCurve.h"
#include "FiberMesh.h"

extern MeshSkeleton * skel;

class Frontend : public InteractionListener {
   public:
      Frontend(OpenGLCanvas *parent);
      Frontend(OpenGLCanvas *parent, const std::string &fileName);
      
      virtual ~Frontend();
      
      // Callbacks upon user interaction
      // Search for Visualization in the Milton docs if you'd like more info
      virtual void mousePressEvent  (InteractionInfo &info);
      virtual void mouseMoveEvent   (InteractionInfo &info);
      virtual void mouseReleaseEvent(InteractionInfo &info);
      virtual void keyPressEvent    (InteractionInfo &info);
      
      /// Called during OpenGL painting in parent Canvas
      virtual void paintGL();
      
      /// Called non-OpenGL painting in parent Canvas
      virtual void paint(QPainter *p);
      
      /// Called upon parent Canvas initialization
      virtual void init();
      
      inline unsigned getWidth() const {
         return m_parent->getWidth();
      }
      
      inline unsigned getHeight() const {
         return m_parent->getHeight();
      }
      
      inline Camera *getCamera() {
         return m_parent->getCamera();
      }

      bool GetVertexClosestToMouse(const Vector2 pt, unsigned int &meshIndex,
                                   unsigned int &vertIndex);


   protected:
      virtual void _handleCameraMousePressEvent(QMouseEvent *event);
      virtual void _handleCameraMouseMoveEvent (QMouseEvent *event);
      
   protected:
      Vector2List     m_active;
      std::vector<FiberMesh *> m_fiberMesh;
      
      // Camera user interaction members
      Point2          m_lastMousePos;
      Point2          m_mouseDownPos;

      std::vector<unsigned>     m_deformVerts;
      
      Point3          m_origin;
      Point3          m_origEye;
      Vector3         m_origUp;
      Vector3         m_origLook;
      Vector3         m_origU;

      bool            m_mouseDown;
      bool            m_curveCreated;
      bool            m_meshCreated;
      bool            m_deforming;

      unsigned int    m_deformingID;
      unsigned int    m_deformingVert;

      ControlCurve    m_activeCurve;

      Mesh            *m_origMesh;
      FiberMesh       *m_contractFMesh;
      
};

#endif // FIBERMESH_FRONTEND_H_

