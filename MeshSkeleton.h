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

class MeshSkeleton {
 public:

  MeshSkeleton();

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
  
};

#endif
