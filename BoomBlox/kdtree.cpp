#include "kdtree.h"

bool cmp(float *p,float *q)
{
	return p[0]<q[0];
}

kdtree::~kdtree(){
	if(this->lc!=nullptr) delete this->lc;
	if(this->rc!=nullptr) delete this->rc;
	//delete(this);
}

void kdtree::release(){
	//if(this->lc!=nullptr) this->lc->~kdtree();
	//if(this->rc!=nullptr) this->rc->~kdtree();
	if(this->lc!=nullptr) this->lc->release();
	if(this->rc!=nullptr) this->rc->release();
	this->~kdtree();
}

kdtree::kdtree(vector<RigidBody *> r,int numOfBody,int *id,int depth){
	if(numOfBody==1){//leaf
		lc=nullptr;
		rc=nullptr;
		index=id[0];

		float radius;
		Vector3 center=r[id[0]]->GetPosition();
		Sphere *sphere = dynamic_cast<Sphere *>(r[id[0]]);
		Box *box=dynamic_cast<Box *>(r[id[0]]);
		Ground *ground=dynamic_cast<Ground *>(r[id[0]]);
		if(sphere) radius=sphere->GetRadius();
		else if(box) radius=box->GetHalfSize().length();
		else radius=1000000;
		//radius+=1e-1;

		xmax=center.x+radius;xmin=center.x-radius;
		ymax=center.y+radius;ymin=center.y-radius;
		zmax=center.z+radius;zmin=center.z-radius;

		return;
	}
	index=-1;

	for(int i=0;i<numOfBody;i++){
		float radius,*tmp;
		Sphere *sphere = dynamic_cast<Sphere *>(r[id[i]]);
		Box *box=dynamic_cast<Box *>(r[id[i]]);
		Ground *ground=dynamic_cast<Ground *>(r[id[i]]);
		if(sphere) radius=sphere->GetRadius();
		else if(box) radius=box->GetHalfSize().length();
		else radius=1000000;
		//radius+=1e-1;

		tmp=new float[2];
		tmp[1]=id[i];
		vxmin.push_back(tmp);
		vxmin[i][0]=r[id[i]]->GetPosition().x-radius;
		tmp=new float[2];
		tmp[1]=id[i];
		vxmax.push_back(tmp);
		vxmax[i][0]=r[id[i]]->GetPosition().x+radius;

		tmp=new float[2];
		tmp[1]=id[i];
		vymin.push_back(tmp);
		vymin[i][0]=r[id[i]]->GetPosition().y-radius;
		tmp=new float[2];
		tmp[1]=id[i];
		vymax.push_back(tmp);
		vymax[i][0]=r[id[i]]->GetPosition().y+radius;

		tmp=new float[2];
		tmp[1]=id[i];
		vzmin.push_back(tmp);
		vzmin[i][0]=r[id[i]]->GetPosition().z-radius;
		tmp=new float[2];
		tmp[1]=id[i];
		vzmax.push_back(tmp);
		vzmax[i][0]=r[id[i]]->GetPosition().z+radius;
	}
	/*for(int i=0;i<vxmin.size();i++)
		std::cout<<vxmin[i][0]<<std::endl;
	for(int i=0;i<vxmin.size();i++)
		std::cout<<vxmin[i][0]<<std::endl;
	getchar();*/
	std::sort(vxmin.begin(),vxmin.end(),cmp);
	std::sort(vxmax.begin(),vxmax.end(),cmp);
	std::sort(vymin.begin(),vymin.end(),cmp);
	std::sort(vymax.begin(),vymax.end(),cmp);
	std::sort(vzmin.begin(),vzmin.end(),cmp);
	std::sort(vzmax.begin(),vzmax.end(),cmp);
	xmin=vxmin[0][0];xmax=vxmax[numOfBody-1][0];
	ymin=vymin[0][0];ymax=vymax[numOfBody-1][0];
	zmin=vzmin[0][0];zmax=vzmax[numOfBody-1][0];
	//for(int i=0;i<vxmin.size();i++)
		//std::cout<<vxmin[i][0]<<std::endl;
	//for(int i=0;i<vxmin.size();i++)
		//std::cout<<vxmin[i][0]<<std::endl;
	//getchar();

	lid=new int[numOfBody/2];
	rid=new int[numOfBody-numOfBody/2];
	if(depth%2==0){
		for(int i=0;i<numOfBody/2;i++) lid[i]=vxmin[i][1];
		for(int i=0;i<numOfBody-numOfBody/2;i++) rid[i]=vxmin[i+numOfBody/2][1];
	}
	/*else if(depth%3==1){
		for(int i=0;i<numOfBody/2;i++) lid[i]=vymin[i][1];
		for(int i=0;i<numOfBody-numOfBody/2;i++) rid[i]=vymin[i+numOfBody/2][1];
	}*/
	else if(depth%2==1){
		for(int i=0;i<numOfBody/2;i++) lid[i]=vzmin[i][1];
		for(int i=0;i<numOfBody-numOfBody/2;i++) rid[i]=vzmin[i+numOfBody/2][1];
	}

	lc=new kdtree(r,numOfBody/2,lid,depth+1);
	rc=new kdtree(r,numOfBody-numOfBody/2,rid,depth+1);

	for(int i=0;i<numOfBody;i++){
		delete[] vxmin[i];
		delete[] vxmax[i];
		delete[] vymin[i];
		delete[] vymax[i];
		delete[] vzmin[i];
		delete[] vzmax[i];
	}
	vxmin.clear();vxmax.clear();
	vymin.clear();vymax.clear();
	vzmin.clear();vzmax.clear();
	delete[] lid;
	delete[] rid;
	//delete[] tmp;
}

void kdtree::findBody(kdtree *root,vector<int> &intersect,float xmin,float xmax,float ymin,float ymax,float zmin,float zmax,int n){
	float ep=0;
	if(root->index==-1){
		if(xmin-ep<root->xmax&&xmax+ep>root->xmin&&ymin-ep<root->ymax&&ymax+ep>root->ymin&&zmin-ep<root->zmax&&zmax+ep>root->zmin){
			findBody(root->lc,intersect,xmin,xmax,ymin,ymax,zmin,zmax,n);
			findBody(root->rc,intersect,xmin,xmax,ymin,ymax,zmin,zmax,n);
		}
	}
	else if(root->index>n){
		if(xmin-ep<root->xmax&&xmax+ep>root->xmin&&ymin-ep<root->ymax&&ymax+ep>root->ymin&&zmin-ep<root->zmax&&zmax+ep>root->zmin)
			intersect.push_back(root->index);
		/*if(n==1){
			std::cout<<"index:"<<root->index<<std::endl;
			if(root->index==9){
				std::cout<<"xmin:"<<xmin<<std::endl;
				std::cout<<"root->xmax:"<<root->xmax<<std::endl;
				std::cout<<"xmax:"<<xmax<<std::endl;
				std::cout<<"root->xmin:"<<root->xmin<<std::endl;
			}
		}*/
	}
}