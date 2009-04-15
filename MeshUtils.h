/**<!-------------------------------------------------------------------->
   @class  MeshUtils
   @author Travis Fischer (fisch0920@gmail.com)
   @date   Spring 2009
   
   @brief
      Utility methods for the FiberMesh project that will make your life 
   *much* easier -- note, you are in no way required to use any of these 
   utilities, but their implementations have been thoroughly tested and 
   are fairly efficient.
   <!-------------------------------------------------------------------->**/

#ifndef MESH_UTILS_H_
#define MESH_UTILS_H_

#include <shapes/Mesh.h>
#include <set>

DECLARE_STL_TYPEDEF(std::vector<Vector2>,  Vector2List);
DECLARE_STL_TYPEDEF(std::vector<Vector3>, Vector3List);

class Camera;

/**
 * Stores neighborhoods that give relationships between connected vertices 
 * and triangles
 */
struct Neighborhood {
   // Indices of all adjacent triangles
   unsigned  noTriangles;
   unsigned *triangles;
   
   // Indices of all adjacent vertices
   unsigned  noVertices;
   unsigned *vertices;

   inline Neighborhood() 
      : noTriangles(0), triangles(NULL), 
        noVertices(0), vertices(NULL)
   { }
   
   ~Neighborhood() {
      safeDeleteArray(triangles);
      safeDeleteArray(vertices);
   }
};

class MeshUtils {
   public:
      /**
       * @brief
       *    Projects a 2D point in NDC (normalized device coordinates, ranging in  
       * [0,1]^2) into world-space
       * 
       * @note this method is independent of canvas resolution, and you should 
       *    be passing in 'normalized' pixel coordinates between 0 and 1
       */
       static Vector3 getWorldVertex(const Camera *camera, const Vector2 &filmPt);
      
      /**
       * @returns whether or not the given point @p p lies within the polygon 
       *    defined by the given @p curve
       */
       static bool isPointInPolygon(const Vector2 &p, 
                                    const Vector2List &curve);
      
      /**
       * @brief
       *    Efficiently computes all possible vertex and triangle neighborhoods 
       * for the given mesh. This essentially gives you everything you could 
       * want to know about the *connectivity* of the mesh. 
       * 
       * For a mesh with n vertices, vertexN will be an n-length array indexed 
       * by vertex, where for each vertex, there will be a single Neighborhood 
       * struct, containing the indices of all vertices adjacent to that 
       * vertex and each triangle incident on that vertex.
       * 
       * For a mesh with m triangles, triangleN will be an m-length array 
       * indexed by triangle, where for each triangle, there will be a single 
       * Neighborhood struct, containing the indices of all vertices in 
       * triangles adjacent to the current triangle and the indices of all 
       * triangles adjacent to the current triangle.
       * 
       * @note triangleN is optional and you will likely have no use for triangleN
       *    in FiberMesh
       */
      static bool getNeighbors(Mesh *mesh, 
                               Neighborhood **vertexN, 
                               Neighborhood **triangleN = NULL);
      
      /**
       * @brief
       *    Computes the Delaunay triangulation of a set of points
       * 
       * @note invokes nox Matlab as a subprocess and utilizes its efficient 
       *    implementation of Delaunay triangulation
       * 
       * @returns whether or not the Delaunay triangulation was successful, 
       *    and upon success, @p outTriangles will be filled with the resulting
       *    triangles
       */
      static bool triangulate(const Vector2List &vertices, 
                              std::vector<MeshTriangle> &outTriangles);
      
      /**
       * @brief
       *    Computes the shortest path from the given starting vertex @p 
       * fromVertex to all other vertices in the mesh, where distance between 
       * two vertices is defined as an approximate distance along the surface 
       * of the mesh (Euclidian distance)
       * 
       * @returns the parent vertex for each of the n vertices in @p P, from 
       *    which you can backtrack to find the shortest path to any given 
       *    vertex
       */
      static void computeVertexShortestPaths(const Mesh *mesh, 
                                             const Neighborhood *vertexNeighborhood, 
                                             unsigned fromVertex, 
                                             vector<unsigned> &P);
      
      /**
       * @brief
       *    Computes the shortest path from the given starting vertex @p 
       * fromVertex to all other vertices in the mesh, where distance between 
       * two vertices is defined as an approximate distance along the surface 
       * of the mesh (Euclidian distance)
       * 
       * @returns the parent vertex for each of the n vertices in @p P, from 
       *    which you can backtrack to find the shortest path to any given 
       *    vertex. Also returns the cumulative distances from @p fromVertex 
       *    to each of the n vertices in @p D
       */
      static void computeVertexShortestPaths(const Mesh *mesh, 
                                             const Neighborhood *vertexNeighborhood, 
                                             unsigned fromVertex, 
                                             vector<unsigned> &P, 
                                             vector<double> &D);
      
   private:
      /// used internally by MeshUtils::getNeighbors
      static void _createArrayFromSet(const std::set<unsigned> &toAdd, 
                                      unsigned **array, unsigned *size);
};

#endif // MESH_UTILS_H_

