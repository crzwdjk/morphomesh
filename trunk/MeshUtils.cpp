/**<!-------------------------------------------------------------------->
   @file   MeshUtils.cpp
   @author Travis Fischer (fisch0920@gmail.com)
   @date   Spring 2009
   
   @brief
      Utility methods for the FiberMesh project that will make your life 
   *much* easier -- note, you are in no way required to use any of these 
   utilities, but their implementations have been thoroughly tested and 
   are fairly efficient.
   <!-------------------------------------------------------------------->**/

#include "MeshUtils.h"
#include <milton.h>
#include <queue>
#include <map>
using namespace std;

typedef pair<double, unsigned> pairDI_t;

// projects a 2D point in NDC (normalized device coordinates, ranging in  
// [0,1]^2) into world-space
// note: this method is independent of canvas resolution, and you should 
// be passing in 'normalized' pixel coordinates between 0 and 1
Vector3 MeshUtils::getWorldVertex(const Camera *camera, const Vector2 &filmPt) {
   const Ray &ray = camera->getWorldRay(Point2(filmPt[0], filmPt[1]));
   
   return Vector3((ray.origin + ray.direction).data);
}

bool MeshUtils::getNeighbors(Mesh *mesh, 
                             Neighborhood **vertexN, 
                             Neighborhood **triangleN)
{
   unsigned noVertices    = mesh->getNoVertices();
   unsigned noTriangles   = mesh->getNoTriangles();
   MeshTriangle *triangles = mesh->getTriangles();
   ASSERT(vertexN);
   
   Neighborhood *vertexNeighborhood = new Neighborhood[noVertices];
   *vertexN  = vertexNeighborhood;
   
   // map from vertex index to a set containing indices of triangles which 
   // contain that vertex
   map<unsigned, set<unsigned> > vertexDegrees;
   
   for(unsigned t = noTriangles; t--;) {
      const MeshTriangle &tri = triangles[t];

      for(unsigned j = 3; j--;)
         vertexDegrees[tri.data[j]].insert(t);
   }
   
   // Handle vertex neighborhoods
   for(unsigned v = noVertices; v--;) {
      set<unsigned>::const_iterator iter;
      set<unsigned> vertNeighbors;

      // triangle neighbors of current vertex
      _createArrayFromSet(vertexDegrees[v],
                          &vertexNeighborhood[v].triangles, 
                          &vertexNeighborhood[v].noTriangles);
      
      // Loop through all neighboring triangles of current vertex
      for(unsigned i = vertexNeighborhood[v].noTriangles; i--;) {
         const MeshTriangle &tri = triangles[vertexNeighborhood[v].triangles[i]];
         
         // For all vertices of the current neighboring triangle
         for(unsigned j = 3; j--;) {
            if (tri.data[j] != v)
               vertNeighbors.insert(tri.data[j]);
         }
      }
      
      _createArrayFromSet(vertNeighbors,
                          &vertexNeighborhood[v].vertices, 
                          &vertexNeighborhood[v].noVertices);
   }
   
   if (triangleN) {
      Neighborhood *triangleNeighborhood = new Neighborhood[noTriangles];
      *triangleN = triangleNeighborhood;
      
      // Handle triangle neighborhoods
      for(unsigned t = noTriangles; t--;) {
         set<unsigned> triNeighbors, vertNeighbors;
         const MeshTriangle &tri = triangles[t];
         
         // Loop through each vertex of current triangle
         for(unsigned j = 3; j--;) {
            const Neighborhood &vertNeighbors = vertexNeighborhood[tri.data[j]];
            const unsigned *triNeighborsOfJ = vertNeighbors.triangles;
            
            // Loop through all neighboring triangles of current vertex
            for(unsigned k = vertNeighbors.noTriangles; k--;) {
               unsigned neighborIndex = triNeighborsOfJ[k];
               
               if (neighborIndex != t)
                  triNeighbors.insert(neighborIndex);
            }
         }
         
         _createArrayFromSet(triNeighbors, 
                             &triangleNeighborhood[t].triangles, 
                             &triangleNeighborhood[t].noTriangles);
         
         const Neighborhood &triNeighborsOfT = triangleNeighborhood[t];
         for(unsigned i = triNeighborsOfT.noTriangles; i--;) {
            unsigned neighborIndex = triNeighborsOfT.triangles[i];
            
            bool add = true;
            for(unsigned j = 3; j--;)
               add = (add && (neighborIndex != tri.data[j]));
            
            if (add) {
               // Add vertices of current triangle to vertex neighbors
               for(unsigned k = 3; k--;)
                  vertNeighbors.insert(triangles[neighborIndex].data[k]);
            }
         }
         
         _createArrayFromSet(vertNeighbors, 
                             &triangleNeighborhood[t].vertices, 
                             &triangleNeighborhood[t].noVertices);
      }
   }
   
   return true;
}

void MeshUtils::_createArrayFromSet(const set<unsigned> &toAdd, 
                                    unsigned **array, unsigned *size)
{
   set<unsigned>::const_iterator iter;
   
   *size  = toAdd.size();
   *array = new unsigned[*size];
   unsigned *arr = *array;
   
   for(iter = toAdd.begin(); iter != toAdd.end(); iter++)
      *arr++ = *iter;
}

bool MeshUtils::isPointInPolygon(const Vector2 &p, 
                                 const Vector2List &curve)
{
   const unsigned n = curve.size();
   bool odd = false;
   
   for(unsigned i = 0, j = n - 1; i < n; j = i++) {
      if (((curve[i][1] < p[1] && curve[j][1] >= p[1]) || 
           (curve[j][1] < p[1] && curve[i][1] >= p[1])) && 
          (curve[i][0] + (p[1] - curve[i][1]) * (curve[j][0] - curve[i][0]) / 
           (curve[j][1] - curve[i][1]) < p[0]))
      {
         odd = !odd;
      }
   }
   
   return odd;
}

void MeshUtils::computeVertexShortestPaths(const Mesh *mesh, 
                                           const Neighborhood *vertexNeighborhood, 
                                           unsigned fromVertex, 
                                           vector<unsigned> &P)
{
   vector<double> D;
   
   MeshUtils::computeVertexShortestPaths(mesh, vertexNeighborhood, fromVertex, P, D);
}

void MeshUtils::computeVertexShortestPaths(const Mesh *mesh, 
                                           const Neighborhood *vertexNeighborhood, 
                                           unsigned fromVertex, 
                                           vector<unsigned> &P, 
                                           vector<double> &D)
{
   const unsigned noVertices = mesh->getNoVertices();
   const Vector3 *vertices   = mesh->getVertices();
   
   P.clear();
   P.resize(noVertices, -1);
   
   D.clear();
   D.resize(noVertices, INFINITY);
   
   // reverse comparator so that Q.top() returns the smallest element
   priority_queue<pairDI_t, vector<pairDI_t>, greater<pairDI_t> > Q;
   Q.push(pairDI_t(0.0, fromVertex));
   D[fromVertex] = 0.0;
   
   // Dijkstra's algorithm
   while(!Q.empty()) {
      const unsigned curVert = Q.top().second;
      const double dist = Q.top().first;
      Q.pop();
      
      const Neighborhood &neighbors = vertexNeighborhood[curVert];
      
      // this check handles duplicates in the queue
      if (dist <= D[curVert]) {
         for(unsigned i = neighbors.noVertices; i--;) {
            const unsigned curNeighbor = neighbors.vertices[i];
            const double cost = (vertices[curVert] - vertices[curNeighbor]).getMagnitude();
            
            if (D[curVert] + cost < D[curNeighbor]) {
               D[curNeighbor] = D[curVert] + cost;
               Q.push(pairDI_t(D[curNeighbor], curNeighbor));
               P[curNeighbor] = curVert;
            }
         }
      }
   }
}

bool MeshUtils::triangulate(const Vector2List &vertices, 
                            std::vector<MeshTriangle> &outTriangles)
{
   const std::string &prefix  = "triangulateFibermesh";
   const std::string &inFile  = prefix + ".m";
   const std::string &outFile = prefix + ".mat";
   const unsigned n = vertices.size();
   
   { // setup matlab file to execute delaunay triangulation
      std::ofstream out;
      out.open(inFile.c_str());
      
      if (!out.is_open()) {
         cerr << "error opening file '" << inFile << "'" << endl;
         return false;
      }
      
      out << "y = [ ";
      for(unsigned i = 0; i < n; ++i)
         out << vertices[i][1] << " ";
      out << "]; x = [ ";
      for(unsigned i = 0; i < n; ++i)
         out << vertices[i][0] << " ";
      out << " ]; t = delaunay(x,y); save('" << outFile << "', 't', '-ASCII'); quit;" << endl;
   }
   
   { // invoke matlab :-)
      const std::string &command = "matlab -nodisplay -r " + prefix;
      FILE *f = popen(command.c_str(), "r");
      if (NULL == f)
         return false;
      
      if (pclose(f))
         return false;
   }
   
   { // parse triangles from matlab output
      std::ifstream in;
      in.open(outFile.c_str());
      
      if (!in.is_open()) {
         cerr << "error opening file '" << in << "'" << endl;
         return false;
      }
      
      outTriangles.clear();
      
      do {
         double aIn, bIn, cIn;
         in >> aIn >> bIn >> cIn;
         
         if (in.eof() || in.bad() || in.fail())
            break;
         
         // a, b, and c are the vertex indices of current triangle
         unsigned a = aIn - 1;
         unsigned b = bIn - 1;
         unsigned c = cIn - 1;
         
         // ensure indices are within valid bounds
         ASSERT(a < n && b < n && c < n);
         
         outTriangles.push_back(MeshTriangle(b, a, c));
      } while(1);
   }
   
   { // remote temporary files
      remove(inFile.c_str());
      remove(outFile.c_str());
   }
   
   return true;
}

