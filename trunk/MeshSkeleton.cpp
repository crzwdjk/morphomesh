#include <cstdio>
#include "MeshSkeleton.h"

MeshSkeleton * MeshSkeleton::fromFile(const char * filename)
{
    FILE * f = fopen(filename, "r");
    if (f == NULL) return NULL;

    MeshSkeleton * skel = new MeshSkeleton;

    int c;
    while ((c = fgetc(f)) != EOF) {
	Vector3 p;
	Bone b;
	switch (c) {
	case 'v':
	    fscanf(f, "%lf %lf %lf\n", &(p[0]), &(p[1]), &(p[2]));
	    printf("v %lf %lf %lf\n", p[0], p[1], p[2]);
	    skel->m_nodes.push_back(p);
	    break;
	case 'e':
	    fscanf(f, "%d %d\n", &(b.start_node), &(b.end_node));
	    printf("e %d %d\n", b.start_node, b.end_node);
	    skel->m_bones.push_back(b);
	}
    }
    skel->initTetrabones();
    return skel;
}

void MeshSkeleton::initTetrabones()
{
    // XXX: fuzzy math? is this right?
    for (unsigned i = 0; i < m_bones.size(); i++) {
	Vector3 v1 = m_nodes[m_bones[i].start_node];
	Vector3 v2 = m_nodes[m_bones[i].end_node];
	Vector3 d = v2 - v1;
	Vector3 q; q[0] = d[1]; q[1] = -d[2]; q[2] = d[0];
	Vector3 qproj = (q - d * d.dot(q)).getNormalized();
	m_bones[i].v3 = 0.5 * (v1 + v2) + qproj * d.getMagnitude() * sqrt(0.75);
	Vector3 c = (1 / 3.0) * (v1 + v2 + m_bones[i].v3);
	m_bones[i].v4 = c + d.cross(m_bones[i].v3 - v1).getNormalized() * sqrt(2.0 / 3.0) * d.getMagnitude();
    }
}

static const double colors[][3] = 
    { {1, 0, 0},
      {0, 1, 0},
      {0, 0, 1},
      {1, 1, 0},
      {0, 1, 1},
      {0.7, 0, 0.7}, };

void MeshSkeleton::draw()
{
    // draw bones
    glLineWidth(3);
    glBegin(GL_LINES);
    for (unsigned i = 0; i < m_bones.size(); i++) {
	glColor3dv(colors[i % 6]);
	glVertex3dv(m_nodes[m_bones[i].start_node].data);
	glVertex3dv(m_nodes[m_bones[i].end_node].data);
    }
    glEnd();
    // draw tetrabones
    glLineWidth(0.7);
    glBegin(GL_LINES);
    for (unsigned i = 0; i < m_bones.size(); i++) {
	glColor3dv(colors[i % 6]);
	glVertex3dv(m_nodes[m_bones[i].start_node].data);
	glVertex3dv(m_bones[i].v3.data);

	glVertex3dv(m_nodes[m_bones[i].end_node].data);
	glVertex3dv(m_bones[i].v3.data);

	glVertex3dv(m_nodes[m_bones[i].start_node].data);
	glVertex3dv(m_bones[i].v4.data);
	
	glVertex3dv(m_nodes[m_bones[i].end_node].data);
	glVertex3dv(m_bones[i].v4.data);

	glVertex3dv(m_bones[i].v3.data);
	glVertex3dv(m_bones[i].v4.data);
    }
    glEnd();

}

void MeshSkeleton::bindMesh(Mesh * m)
{
    // for each vertex v_i in Mesh 
    //   find the nearest point on a bone
    //   if that point is not a joint:
    //     set w[i][b] = 1
    //   else:
    //     set w[i][b1] = w[i][b2] = 0.5
}

// call this after you've adjusted the positions of the bones.
void MeshSkeleton::update()
{
    // 
}
