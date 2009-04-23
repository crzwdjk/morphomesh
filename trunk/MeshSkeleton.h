/*************************************************************
 *  Trevor O'Brien, trevor@cs.brown.edu
 *
 *  April 11, 2009
 *  MeshSkeleton.h
 *
 ************************************************************/


#ifndef MESHSKELETON_H
#define MESHSKELETON_H

#include <milton.h>

/* A mapping from a mesh vertex to its corresponding skeleton bone
   specifies relative position in cylindrical coordinates 
   (XXX: not sure this is a good idea) */
struct SkeletonMapping {
  int bone;          // the index of the skeleton bone
  double t;          // distance along bone, range [0, 1]
  double rotation;   // rotation around bone, range [0, 2pi)
  double distance;   // distance from bone
};


stuct Bone {
  int start_node;
  int end_node;
  Vector3 orient;    // basis for detrmining rotation about bone
};

class MeshSkeleton {
 public:

  MeshSkeleton(Mesh & m) : m_mesh(m) {  }

  virtual ~MeshSkeleton();

  void AddNode(Vector3 nodePos);
  
  void RemoveNode(unsigned int nodeIndex);

  std::vector<Vector3> GetNodes() {
    return m_nodes;
  }

  unsigned int GetNumNodes() {
    return m_nodes.size();
  }

 private:
  std::vector<Vector3> m_nodes;
  std::vector<Bone> m_bones;
  
  std::hash_map<int, SkeletonMapping> m_correspond;
  Mesh & m_mesh;
};

#endif
