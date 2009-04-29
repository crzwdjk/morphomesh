#include <cstdio>
#include "MeshSkeleton.h"

MeshSkeleton * MeshSkeleton::fromFile(const char * filename)
{
    FILE * f = fopen(filename, "r");
    if (f == NULL) return NULL;

    MeshSkeleton * skel = new MeshSkeleton;

    char line[80];
    while (fgets(line, 80, f)) {
	Vector3 p;
	Bone b;
	switch (line[0]) {
	case 'v':
	    sscanf(line + 1, "%lf %lf %lf\n", &(p[0]), &(p[1]), &(p[2]));
	    printf("v %lf %lf %lf\n", p[0], p[1], p[2]);
	    skel->m_nodes.push_back(p);
	    break;
	case 'e':
	    sscanf(line + 1, "%d %d\n", &(b.start_node), &(b.end_node));
	    printf("e %d %d\n", b.start_node, b.end_node);
	    skel->m_bones.push_back(b);
	    break;
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
    return;

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

static Vector3 closest_point_seg(Vector3 p, Vector3 seg_start, Vector3 seg_end)
{
    Vector3 ab = seg_end - seg_start;
    Vector3 ac = p - seg_start;
    
    double t = ab.dot(ac)/ab.getMagnitude2();
    if (t <= 0) return seg_start;
    else if (t >= 1) return seg_end;
    else return t * ab + seg_start;
}

void MeshSkeleton::bindMesh(Mesh * m)
{
    //Store mesh
    m_mesh = m;

    //Create weight matrix
    weights = new SparseMatrix((m_nodes.size()+m->getNoVertices()), m_bones.size());

    //Bind skeleton nodes to bones
    for (unsigned n = 0; n < m_nodes.size(); n++) {
	const Vector3 v = m_nodes[n];

	int closest_bone = -1;
	double closest_distance = INFINITY;
	Vector3 closest_point;
	for (unsigned b = 0; b < m_bones.size(); b++) {
	    Vector3 p = closest_point_seg(m_nodes[n], 
					  m_nodes[m_bones[b].start_node],
					  m_nodes[m_bones[b].end_node]);
	    const double d = p.getDistance(v);
	    if (d < closest_distance) {
		closest_bone = b;
		closest_distance = d;
		closest_point = p;
	    }
	}
	// handle joint case
	if (closest_point == m_nodes[m_bones[closest_bone].start_node]
	    || closest_point == m_nodes[m_bones[closest_bone].end_node]) {
	    int bonecount = 0;
	    for (unsigned b1 = 0; b1 < m_bones.size(); b1++) {
		if (closest_point == m_nodes[m_bones[b1].start_node] ||
		    closest_point == m_nodes[m_bones[b1].end_node]) {
		    bonecount++;
		    weights->setValue(n, b1, 1.0);
		}
	    }
	    // normalize
	    for(int i = 0; i < m_bones.size(); i++)
		weights->setValue(n, i, weights->getValue(n, i) / bonecount);
	} 
	else {
	    weights->setValue(n, closest_bone, 1.0);
	}
    } //Done binding skeleton nodes to bones

    //Bind mesh vertices to bones
    Vertex* vertices = m->getVertices();
    for (unsigned k = 0; k < m->getNoVertices(); k++) {
	const Vector3 v = vertices[k];

	int closest_bone = -1;
	double closest_distance = INFINITY;
	Vector3 closest_point;
	for (unsigned b = 0; b < m_bones.size(); b++) {
	    Vector3 p = closest_point_seg(vertices[k], 
					  m_nodes[m_bones[b].start_node],
					  m_nodes[m_bones[b].end_node]);
	    const double d = p.getDistance(v);
	    if (d < closest_distance) {
		closest_bone = b;
		closest_distance = d;
		closest_point = p;
	    }
	}
	// handle joint case
	if (closest_point == m_nodes[m_bones[closest_bone].start_node]
	    || closest_point == m_nodes[m_bones[closest_bone].end_node]) {
	    int bonecount = 0;
	    for (unsigned b1 = 0; b1 < m_bones.size(); b1++) {
		if (closest_point == m_nodes[m_bones[b1].start_node] ||
		    closest_point == m_nodes[m_bones[b1].end_node]) {
		    bonecount++;
		    weights->setValue(k+m_nodes.size(), b1, 1.0);
		}
	    }
	    // normalize
	    for(int i = 0; i < m_bones.size(); i++)
		weights->setValue(k+m_nodes.size(), i, weights->getValue(k+m_nodes.size(), i) / bonecount);
	} 
	else {
	    weights->setValue(k+m_nodes.size(), closest_bone, 1.0);
	}
    } //Done binding mesh to skeleton

    
}

// call this after you've adjusted the positions of the bones.
void MeshSkeleton::update()
{
   
}
