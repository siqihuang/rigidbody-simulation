#ifndef _KDTREE_
#define _KDTREE_

#include <vector>
#include "RigidBody.h"
#include "Sphere.h"
#include "Box.h"
#include "Ground.h"
#include <algorithm>
#include <iostream>
using namespace std;

class kdtree{
public:
	int index,depth,*lid,*rid;
	float xmin,xmax,ymin,ymax,zmin,zmax;
	kdtree *lc,*rc;
	std::vector<float *> vxmin,vxmax,vymin,vymax,vzmin,vzmax;

	kdtree(vector<RigidBody *> r,int numOfBody,int *id,int depth);
	~kdtree();
	void release();
	void findBody(kdtree *root,vector<int> &intersect,float xmin,float xmax,float ymin,float ymax,float zmin,float zmax,int n);
};

#endif