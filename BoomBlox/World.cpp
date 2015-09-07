#include <OgreMath.h>
#include "World.h"
#include "RigidBody.h"
#include "TestHarness.h"
#include "Box.h"
#include "Sphere.h"
#include "Ground.h"
#include "Material.h"
#include <algorithm>
#include <queue>
#include <map>

World::World()
{
	frame=0;
	root=nullptr;
}

World::~World()
{
	Clear();
}

World::World(World const& other)
{
	root=nullptr;
	FillFrom(other);
}

World & World::operator=(World const& other)
{
	if(this == &other) return *this;
	Clear();
	FillFrom(other);
	return *this;
}

void World::FillFrom(World const& other)
{
	std::map<Material const*, Material*> materialMapping;

	for(int i=0; i<other.GetNumMaterials(); i++)
	{
		Material const* oldMaterial = other.GetMaterial(i);
		Material* newMaterial = new Material(*oldMaterial);
		materialMapping[oldMaterial] = newMaterial;
		AddMaterial(newMaterial);
	}

	for(int i=0; i<other.GetNumBodies(); i++)
	{
		RigidBody const* oldBody = other.GetBody(i);
		RigidBody* newBody = oldBody->Clone();
		newBody->SetMaterial(materialMapping[oldBody->GetMaterial()]);
		AddBody(newBody);
	}
}

void World::AddMaterial(Material* material)
{
	m_materials.push_back(material);
}

int World::GetNumMaterials() const
{
	return int(m_materials.size());
}

Material const* World::GetMaterial(int material) const
{
	return m_materials[material];
}

void World::Clear()
{
	for(int i=0; i<GetNumBodies(); i++)
	{
		delete GetBody(i);
	}

	for(int i=0; i<GetNumMaterials(); i++)
	{
		delete GetMaterial(i);
	}

	m_bodies.clear();
	m_materials.clear();
}

void World::AddBody(RigidBody* body)
{
	m_bodies.push_back(body);
}

int World::GetNumBodies() const
{
	return int(m_bodies.size());
}

RigidBody* World::GetBody(int body)
{
	return m_bodies[body];
}

RigidBody const* World::GetBody(int body) const
{
	return m_bodies[body];
}

int World::FindBody(RigidBody* body) const
{
	for(int i=0; i<GetNumBodies(); i++)
	{
		if(GetBody(i) == body)
		{
			return i;
		}
	}

	return -1;
}

void World::RemoveBody(int body)
{
	m_bodies.erase(m_bodies.begin()+body);
}

void World::Render() const
{
	for(int i=0; i<GetNumBodies(); i++)
	{
		GetBody(i)->Render();
	}
}

void World::Simulate(float dT)
{
	// LOOK this is the basis of simulation
	ResolveCollisions(dT);
	AdvanceVelocities(dT);
	ResolveContacts(dT);
	AdvancePositions(dT);
}

void World::ResolveCollisions(float dT)
{
	const int NUM_ITERATIONS = 5; // LOOK many iterations. Some things are easier to troubleshoot with only one.
	for(int ii=0; ii<NUM_ITERATIONS; ii++)
	{
		// test iterating forward
		SaveState();
		AdvanceVelocities(dT);
		AdvancePositions(dT);
		std::vector<Intersection> intersections;
		FindIntersections(intersections);
		RestoreState();
		if(intersections.empty())
		{
			break;
		}

		for(int ic=0; ic<int(intersections.size()); ic++)
		{
			Intersection &i = intersections[ic];
			float epsilon = std::min(i.bodyA->GetMaterial()->restitution, i.bodyB->GetMaterial()->restitution);
			ResolveIntersection(i, epsilon);
		}

		bool hadImpulses = false;
		for(int i=0; i<GetNumBodies(); i++)
		{
			hadImpulses = GetBody(i)->ApplyQueuedImpulses() || hadImpulses;
		}
		if(!hadImpulses)
		{
			break; // We stop early if nothing changed
		}
	}
}

TEST(AdvanceVelocities, World)
{
	Box* b = new Box(Vector3(1,1,1));
	b->SetVelocity(Vector3(0,10,0));
	World w;
	w.AddBody(b);

	w.AdvanceVelocities(1);
	std::cout<<b->GetVelocity()<<std::endl;
	CHECK(b->GetVelocity().positionEquals(Vector3(0, 0.2f, 0)));
}

void World::AdvanceVelocities(float dT)
{
	for(int i=0; i<GetNumBodies(); i++)
	{
		GetBody(i)->AdvanceVelocity(Vector3(0,-9.8,0),dT);
	}
}

void World::ResolveContacts(float dT)
{
	const int NUM_RELAXATION_STEPS = 50;
	const int NUM_PARTIAL_RELAXATION_STEPS = 10;

	for(int ir=0; ir<NUM_RELAXATION_STEPS; ir++)
	{
		SaveState();
		AdvancePositions(dT);
		std::vector<Intersection> intersections;
		FindIntersections(intersections);
		RestoreState();
		if(intersections.empty())
		{
			break;
		}

		for(int ic=0; ic<int(intersections.size()); ic++)
		{
			Intersection &i = intersections[ic];
			ResolveIntersection(i, -1 + std::min(1.0f, float(ir)/NUM_PARTIAL_RELAXATION_STEPS), true);
		}
	}
}

TEST(AdvancePositions, World)
{
	Box* b = new Box(Vector3(1,1,1));
	b->SetPosition(Vector3(0,50,0));
	b->SetVelocity(Vector3(0,-2,0));
	b->SetAngularVelocity(Vector3(0,0,0.01f));
	World w;
	w.AddBody(b);

	w.AdvancePositions(2);
	CHECK(b->GetPosition().positionEquals(Vector3(0, 46, 0)));
	float angle;
	Vector3 axis;
	b->GetOrientation().ToAngleAxis(angle, axis);
	CHECK(axis.positionEquals(Vector3(0,0,1)));
	FLOATS_EQUAL(0.02f, angle);
}

void World::AdvancePositions(float dT)
{
	for(int i=0; i<GetNumBodies(); i++)
	{
		GetBody(i)->AdvanceTransformation(dT);
	}
}

namespace
{
	Matrix3 CalcK(RigidBody const& body, Vector3 const& r)
	{
		if(body.HasInfiniteMass())
		{
			return Matrix3::ZERO;
		}
		else
		{
			Matrix3 rstar(
				0, -r.z, r.y,
				r.z, 0, -r.x,
				-r.y, r.x, 0);

			Matrix3 Iinv = body.GetInverseInertialTensor();

			return Matrix3::IDENTITY * body.GetInverseMass() + rstar.Transpose() * Iinv * rstar;
		}
	}
}

TEST(ResolveIntersection, World)
{
	Material m(1, 50, 1, Vector3(1,1,1)); // high friction
	Sphere s1(1); 
	s1.SetMaterial(&m);
	s1.SetPosition(Vector3(-3,0,0));
	s1.SetVelocity(Vector3(8,0,0));
	Sphere s2(2); 
	s2.SetMaterial(&m);
	Intersection i(&s1, &s2, Vector3(-2,0,0), Vector3(-1,0,0));
	World::ResolveIntersection(i, 1, true);

	//std::cout<<s1.GetVelocity()<<std::endl;
	//std::cout<<std::endl;
	CHECK(s1.GetVelocity().positionEquals(Vector3(-4.8f,0,0)));
	//std::cout<<s2.GetVelocity()<<std::endl;
	CHECK(s2.GetVelocity().positionEquals(Vector3(3.2f,0,0)));

	s1.SetPosition(Vector3(0,1,0));
	s1.SetVelocity(Vector3(1,-1,0));
	Ground g;
	g.SetMaterial(&m);
	Intersection i2(&s1, &g, Vector3(0,0,0), Vector3(0,1,0));
	World::ResolveIntersection(i2, 1, true);
	CHECK(s1.GetVelocity().positionEquals(Vector3(0.71428f,1,0)));
	CHECK(s1.GetAngularVelocity().positionEquals(Vector3(0,0,-0.71428f)));

	s1.SetVelocity(Vector3(1,-1,0));
	s1.SetAngularVelocity(Vector3::ZERO);
	m.friction = 0; // zero friction
	World::ResolveIntersection(i2, 1, true);
	CHECK(s1.GetVelocity().positionEquals(Vector3(1,1,0)));
	CHECK(s1.GetAngularVelocity().positionEquals(Vector3(0,0,0)));

	s1.SetVelocity(Vector3(1,-1,0));
	s1.SetAngularVelocity(Vector3::ZERO);
	m.friction = 0.1f; // low friction
	World::ResolveIntersection(i2, 1, true);
	//std::cout<<s1.GetVelocity()<<std::endl;
	CHECK(s1.GetVelocity().positionEquals(Vector3(0.8f,1,0)));
	CHECK(s1.GetAngularVelocity().positionEquals(Vector3(0,0,-0.5f)));

}

void World::ResolveIntersection(Intersection &i, float epsilon, bool immediate)
{
	// TODO I've shown you the types of the variables and a good order to calculate them in.
	RigidBody &a = *i.bodyA;
	RigidBody &b = *i.bodyB;
	Vector3 N = i.outVector;
	Vector3 loc = i.contactPoint;
	Vector3 locA = i.bodyA->GetTransformation()*i.contactPointA;
	Vector3 locB = i.bodyB->GetTransformation()*i.contactPointB;
	//Vector3 urel = Vector3::ZERO;
	
	//float ureln = 0;
	Vector3 urel=a.GetVelocityAtPoint(locA) - b.GetVelocityAtPoint(locB);
	float ureln=urel.dotProduct(N);
	if(ureln > 0)
	{
		return;
	}
	//Vector3 urelT = Vector3::ZERO;
	//Vector3 T = Vector3::ZERO;

	Matrix3 Ka = CalcK(a, locA-a.GetPosition());
	Matrix3 Kb = CalcK(b, locB-b.GetPosition());
	Matrix3 KT=Ka+Kb;
	float mu = std::max(a.GetMaterial()->friction, b.GetMaterial()->friction);
	
	// try with sticking collision (zero tangential motion after collision)
	Vector3 uprimerel1 = -epsilon*N.dotProduct(urel)*N;
	//Vector3 j = Vector3::ZERO; // As in paragraph 7, section 1: u'rel = urel + KT*j
	Vector3 j=KT.Inverse()*(uprimerel1-urel);
	
	float jdotN = j.dotProduct(N);
	if((j-jdotN*N).squaredLength() < mu*mu*jdotN*jdotN)
	{
		// sticking collision; j is acceptable
		//std::cout<<"!!!!"<<std::endl;
	}
	else
	{
		float jn = 0,jt;
		Vector3 T=urel-N.dotProduct(urel)*N;
		T.normalise();
		//std::cout<<T<<std::endl;
		//std::cout<<N<<std::endl;
		float tmp=-epsilon*N.dotProduct(urel)-N.dotProduct(urel);
		float tmp1=N.dotProduct(KT*(N-mu*T));
		jn=tmp/tmp1;
		jt=mu*jn;
		//j = Vector3::ZERO;
		j=jn*N-jt*T;
	}

	if(immediate)
	{
		a.ApplyImpulse(j, locA);
		b.ApplyImpulse(-j, locB);
	}
	else
	{
		a.QueueImpulse(j, locA);
		b.QueueImpulse(-j, locB);
	}
}

void World::SaveState()
{
	for(int i=0; i<GetNumBodies(); i++)
	{
		GetBody(i)->SaveTransformation();
		GetBody(i)->SaveVelocity();
	}
}

void World::RestoreState()
{
	for(int i=0; i<GetNumBodies(); i++)
	{
		GetBody(i)->RestoreTransformation();
		GetBody(i)->RestoreVelocity();
	}
}

TEST(FindIntersections, World)
{
	Sphere s1(1); s1.SetPosition(Vector3(-2.5f,0,0));
	Sphere s2(1); s2.SetPosition(Vector3(-1.25f,-1,0));
	Sphere s3(1); s3.SetPosition(Vector3(0,0,0));
	Sphere s4(1); s4.SetPosition(Vector3(1.25f,-1,0));
	Sphere s5(1); s5.SetPosition(Vector3(2.5f,0,0));
	Sphere s6(5); s6.SetPosition(Vector3(0,0,0));
	World w;
	w.AddBody(s1.Clone());
	w.AddBody(s2.Clone());
	w.AddBody(s3.Clone());
	w.AddBody(s4.Clone());
	w.AddBody(s5.Clone());
	w.AddBody(s6.Clone());

	std::vector<Intersection> intersections;
	w.FindIntersections(intersections);
	LONGS_EQUAL(9, intersections.size());
}

float getBoundingValue(RigidBody *r,int type,int i){
	Box * box = dynamic_cast<Box *>(r);
	Vector3 center=r->GetPosition();
	if(box)
	{
		float radius=box->GetHalfSize().length();
		if(type==0) return center.x-radius;
		if(type==1) return center.x+radius;
		if(type==2) return center.y-radius;
		if(type==3) return center.y+radius;
		if(type==4) return center.z-radius;
		if(type==5) return center.z+radius;
	}

	Sphere * sphere = dynamic_cast<Sphere *>(r);
	if(sphere)
	{
		float radius=sphere->GetRadius();
		//if(i==100) std::cout<<center.x-radius<<std::endl;
		if(type==0) return center.x-radius;
		if(type==1) return center.x+radius;
		if(type==2) return center.y-radius;
		if(type==3) return center.y+radius;
		if(type==4) return center.z-radius;
		if(type==5) return center.z+radius;
	}

	Ground * ground = dynamic_cast<Ground *>(r);
	if(ground)
	{
		//float radius=sphere->GetRadius();
		if(type==0) return -1000000;
		if(type==1) return 1000000;
		if(type==2) return 0;
		if(type==3) return 0;
		if(type==4) return -1000000;
		if(type==5) return 1000000;
	}

	return -1;
}

void World::FindIntersections(std::vector<Intersection> & intersections)
{
	// TODO
	int n=GetNumBodies();	
	// LOOK this method is slow and not acceptable for a final project, but it works
	// (slowly) and may help you test other parts
	
	/*for(int i=0;i<n;i++){
		for(int j=i+1;j<n;j++){

			float z1max,z1min,z2max,z2min;
			z1max=getBoundingValue(GetBody(i),5,i);
			z2min=getBoundingValue(GetBody(j),4,j);
			if(z1max<z2min) continue;
			z1min=getBoundingValue(GetBody(i),4,i);
			z2max=getBoundingValue(GetBody(j),5,j);
			if(z2max<z1min) continue;
			
			float x1max,x1min,x2max,x2min;
			x1max=getBoundingValue(GetBody(i),1,i);
			x2min=getBoundingValue(GetBody(j),0,j);
			if(x1max<x2min){
				continue;
			}
			x1min=getBoundingValue(GetBody(i),0,i);
			x2max=getBoundingValue(GetBody(j),1,j);
			if(x2max<x1min){
				continue;
			}

			float y1max,y1min,y2max,y2min;
			y1max=getBoundingValue(GetBody(i),3,i);
			y2min=getBoundingValue(GetBody(j),2,j);
			if(y1max<y2min) continue;
			y1min=getBoundingValue(GetBody(i),2,i);
			y2max=getBoundingValue(GetBody(j),3,j);
			if(y2max<y1min) continue;

			FindIntersection(GetBody(i), GetBody(j), intersections);
			//map[i][j]=true;
		}
	}*/

	/*for(int i=0; i<GetNumBodies(); i++)
	{
		for(int j=i+1; j<GetNumBodies(); j++)
		{
			if(map[i][j])
			FindIntersection(GetBody(i), GetBody(j), intersections);
		}
	}*/
	int *id=new int[n];
	vector<int> intersect;
	for(int i=0;i<n;i++) id[i]=i;
	if(n>8){
		if(root!=nullptr){delete root;}
		root=new kdtree(m_bodies,n,id,0);
		delete[] id;
		//std::cout<<n<<std::endl;
		//std::cout<<root->lc->rc->xmax<<std::endl;
	}
	for(int i=0; i<GetNumBodies(); i++)
	{
		if(n>8){
			float radius;
			Vector3 center=GetBody(i)->GetPosition();
			Sphere *sphere = dynamic_cast<Sphere *>(GetBody(i));
			Box *box=dynamic_cast<Box *>(GetBody(i));
			Ground *ground=dynamic_cast<Ground *>(GetBody(i));
			if(sphere) radius=sphere->GetRadius();
			else if(box) radius=box->GetHalfSize().length();
			else radius=1000000;
			root->findBody(root,intersect,center.x-radius,center.x+radius,center.y-radius,center.y+radius,center.z-radius,center.z+radius,i);
			//intersect.clear();
			//for(int j=i+1;j<n;j++) intersect.push_back(j);
			for(int j=0;j<intersect.size();j++)
				FindIntersection(GetBody(i), GetBody(intersect[j]), intersections);
			//std::cout<<intersect.size()<<std::endl;
			//for(int j=0;j<intersect.size();j++)
				//std::cout<<intersect[j]<<std::endl;

			//std::cout<<GetBody(8)->GetPosition()<<std::endl;
			//getchar();
			intersect.clear();
		}
		else
			for(int j=i+1; j<GetNumBodies(); j++)
			{
				FindIntersection(GetBody(i), GetBody(j), intersections);
			}
	}
	//std::cout<<GetNumBodies()<<std::endl;
}
