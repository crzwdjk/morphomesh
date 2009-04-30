#include <cstdio>
#include "MeshSkeleton.h"


static bool bones_adjacent(vector<Bone> & bones, int b1, int b2)
{
    return bones[b1].start_node == bones[b2].start_node
       || bones[b1].start_node == bones[b2].end_node
       || bones[b1].end_node == bones[b2].start_node
       || bones[b1].end_node == bones[b2].end_node;
}



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

    cerr << "assigning bones" << endl;
    // initialize bone tree
    skel->m_parents.insert(skel->m_parents.begin(), skel->m_bones.size(), -2);
    // XXX: find centroid bone. Default to the first one currently.
    skel->m_parents[0] = -1;
    int boned;
    do {
       boned = 0;
       for (unsigned b_i = 0; b_i < skel->m_bones.size(); b_i++) {
           // if this bone has a parent
           if (skel->m_parents[b_i] != -2) {
               // parent all the adjacent bones to it
               for (unsigned b_j = 0; b_j < skel->m_bones.size(); b_j++) {
                   if (skel->m_parents[b_j] == -2 && bones_adjacent(skel->m_bones, b_i, b_j)) {
                       skel->m_parents[b_j] = b_i;
                       boned++;
                   }
               }
           }
       }
       cerr << "boned " << boned <<endl;
    } while (boned > 0);
    cerr << "assigned" << endl;

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
    //return;

    // draw mesh

    if (m_mesh != NULL) {
	double cr[m_mesh->getNoVertices()];
	double cg[m_mesh->getNoVertices()];
	double cb[m_mesh->getNoVertices()];

	weights->multiply(cr, m_colors[0], m_bones.size(), 1);
	weights->multiply(cg, m_colors[1], m_bones.size(), 1);
	weights->multiply(cb, m_colors[2], m_bones.size(), 1);

	
	glBegin(GL_TRIANGLES);
	for (unsigned ti = 0; ti < m_mesh->getNoTriangles(); ti++) {
	    MeshTriangle t = m_mesh->getTriangles()[ti];
	    int v[3] = {t.A, t.B, t.C};
	    for (int vi = 0; vi < 3; vi++) {
		glColor4d(cr[v[vi]], cg[v[vi]], cb[v[vi]], 0.4);
		glVertex3dv(m_mesh->getVertices()[v[vi]].data);
	    }
	}
	glEnd();
    }

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

// note: this will go away once we have an extractable skeleton
void MeshSkeleton::bindMesh(Mesh * m)
{

    m_mesh = m;
    //Create weight matrix
    weights = new SparseMatrix(m_mesh->getNoVertices(), m_bones.size());

    //Bind mesh vertices to bones
    for (unsigned n = 0; n < m_mesh->getNoVertices(); n++) {
	const Vector3 v = m_mesh->getVertices()[n];
	const Vector3 vn = m_mesh->getNormals()[n];

	int closest_bone = -1;
	double closest_distance = INFINITY;
	Vector3 closest_point;
	for (unsigned b = 0; b < m_bones.size(); b++) {
	    Vector3 p = closest_point_seg(v, 
					  m_nodes[m_bones[b].start_node],
					  m_nodes[m_bones[b].end_node]);
	    double d = p.getDistance(v);
	    // WARNING: this may not be a good feature. feel free to disable.
	    // fudge factor: vertex normal should be pointing away from bone
	    double f = vn.dot((v - p).getNormalized());
	    if (f < 0) 
		d *= 20;
	    if (f < 0.8)
		d *= (0.85 - f) * 20;
	    // END WARNING
	    
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
    } //Done binding mesh vertices to bones

    m_colors[0] = new double[m_bones.size()];
    m_colors[1] = new double[m_bones.size()];
    m_colors[2] = new double[m_bones.size()];
    for (unsigned b = 0; b < m_bones.size(); b++) {
	m_colors[0][b] = colors[b % 6][0];
	m_colors[1][b] = colors[b % 6][1];
	m_colors[2][b] = colors[b % 6][2];
    }
}

// call this after you've adjusted the positions of the bones.
void MeshSkeleton::update(int changed_node, Vector3 newpos)
{
    std::vector<Bone> changed_bones_old;
    std::vector<int> changed_bones_indices;
    m_nodes.push_back(Vector3(m_nodes[changed_node]));
    for(unsigned bone_index=0; bone_index < m_bones.size(); bone_index++) {
        if(m_bones[bone_index].start_node == changed_node || m_bones[bone_index].end_node == changed_node) {
            Bone b;
            if (m_bones[bone_index].end_node == changed_node) {
                b.start_node = m_bones[bone_index].start_node;
                b.end_node = m_nodes.size()-1;
            } else { 
                b.start_node = m_nodes.size()-1;
                b.end_node = m_bones[bone_index].end_node;
            }
            b.v3 = Vector3(m_bones[bone_index].v3);
            b.v4 = Vector3(m_bones[bone_index].v4);
            changed_bones_old.push_back(b);
            changed_bones_indices.push_back(bone_index);
        }
    }

    Vector3 delta = newpos - m_nodes[changed_node];
    // for each bone that is changed
    // subtract the component parallel to that bone from newpos
    for (unsigned b_i = 0; b_i < changed_bones_old.size(); b_i++) {
	Vector3 bonevect = m_nodes[changed_bones_old[b_i].end_node] - m_nodes[changed_bones_old[b_i].start_node];
	bonevect.normalize();
	delta -= bonevect * delta.dot(bonevect);
    }

    m_nodes[changed_node] += delta;
    initTetrabones();

    SparseMatrix T_x = SparseMatrix::zero(m_bones.size(), 4);
    SparseMatrix T_y = SparseMatrix::zero(m_bones.size(), 4);
    SparseMatrix T_z = SparseMatrix::zero(m_bones.size(), 4);
    for(unsigned r=0; r<m_bones.size(); r++) {
        T_x.setValue(r, 0, 1);
        T_y.setValue(r, 1, 1);
        T_z.setValue(r, 2, 1);
    }

    for(unsigned b_i=0; b_i < changed_bones_old.size(); b_i++) {
        Matrix4x4 T_i = extractTransform(changed_bones_old[b_i], m_bones[(changed_bones_indices[b_i])]);
	for(unsigned c=0; c<4; c++) {
            T_x.setValue(changed_bones_indices[b_i], c, T_i[0][c]);
            T_y.setValue(changed_bones_indices[b_i], c, T_i[1][c]);
            T_z.setValue(changed_bones_indices[b_i], c, T_i[2][c]);
         }
    }
    char updated[m_bones.size()];
    memset(updated, 0, m_bones.size());
    updated[changed_node] = 1;
    // bones further out inherit the transfomation of bones closer in
    // WARNING: this may not be a good feature. feel free to disable
    for (unsigned b_i = 0; b_i < m_bones.size(); b_i++) {
        for (unsigned b_j = 0; b_j < changed_bones_old.size(); b_j++) {
            if (m_parents[b_i] == int(changed_bones_indices[b_j])) {
                for (unsigned c = 0; c < 4; c++) {
                    T_x.setValue(b_i, c, T_x.getValue(changed_bones_indices[b_j], c));
                    T_y.setValue(b_i, c, T_y.getValue(changed_bones_indices[b_j], c));
                    T_z.setValue(b_i, c, T_z.getValue(changed_bones_indices[b_j], c));
                }
		if (!updated[m_bones[b_i].start_node]) {
		    m_nodes[m_bones[b_i].start_node] += delta;
		    updated[m_bones[b_i].start_node] = 1;
		}
		if (!updated[m_bones[b_i].end_node]) {
		    m_nodes[m_bones[b_i].end_node] += delta;
		    updated[m_bones[b_i].end_node] = 1;
		}
            }
        }
    }
    // END WARNING

    m_nodes.pop_back();

    T_x = (*weights)*T_x;
    T_y = (*weights)*T_y;
    T_z = (*weights)*T_z;

    unsigned num_vertices = m_mesh->getNoVertices();
    Vertex* vertices = m_mesh->getVertices();
    for (unsigned v_i = 0; v_i < num_vertices; v_i++) {
        Vector3 vertex = vertices[v_i];
        double vector4v[4] = {vertex[0], vertex[1], vertex[2], 1};
        Vector<4,double> vertex_to_transform = Vector<4,double>(vector4v);
        double vector4x[4] = {T_x.getValue(v_i,0), T_x.getValue(v_i,1), T_x.getValue(v_i, 2), T_x.getValue(v_i,3)};
        Vector<4,double> x_transform = Vector<4,double>(vector4x);
        double vector4y[4] = {T_y.getValue(v_i,0), T_y.getValue(v_i,1), T_y.getValue(v_i, 2), T_y.getValue(v_i,3)};
        Vector<4,double> y_transform = Vector<4,double>(vector4y);
        double vector4z[4] = {T_z.getValue(v_i,0), T_z.getValue(v_i,1), T_z.getValue(v_i, 2), T_z.getValue(v_i,3)};
        Vector<4,double> z_transform = Vector<4,double>(vector4z);
        vertices[v_i][0] = vertex_to_transform.dot(x_transform);
        vertices[v_i][1] = vertex_to_transform.dot(y_transform);
        vertices[v_i][2] = vertex_to_transform.dot(z_transform);
    }
}

Matrix4x4 MeshSkeleton::extractTransform(Bone b0, Bone b1) {
    //Extract transformation based on old and new positions
    double tb_original[16] = {m_nodes[b0.start_node][0], m_nodes[b0.end_node][0], b0.v3[0], b0.v4[0], 
			      m_nodes[b0.start_node][1], m_nodes[b0.end_node][1], b0.v3[1], b0.v4[1], 
                              m_nodes[b0.start_node][2], m_nodes[b0.end_node][2], b0.v3[2], b0.v4[2], 
			      1, 1, 1, 1};

    double tb_deformed[16] = {m_nodes[b1.start_node][0], m_nodes[b1.end_node][0], b1.v3[0], b1.v4[0], 
			      m_nodes[b1.start_node][1], m_nodes[b1.end_node][1], b1.v3[1], b1.v4[1], 
			      m_nodes[b1.start_node][2], m_nodes[b1.end_node][2], b1.v3[2], b1.v4[2], 
 			      1, 1, 1, 1};
    Matrix4x4 v_old = Matrix4x4(tb_original);
    Matrix4x4 v_new = Matrix4x4(tb_deformed);     
    return v_new*v_old.getInverse(); 
}
