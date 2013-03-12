#ifndef __PHYSICS_H__
#define __PHYSICS_H__

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/glut.h>
#include <math.h>
#include <iostream>

#include "Vector.h"

#define MAX_BODIES   2048 
#define MAX_VERTICES 5096
#define MAX_EDGES    5096
#define MAX_BODY_VERTICES 128 
#define MAX_BODY_EDGES    128
#define PI 3.14159

struct PhysicsBody;
struct Vertex {
	Vec2 Position;
	Vec2 OldPosition;
	Vec2 Acceleration;

	PhysicsBody* Parent;

	Vertex( PhysicsBody* Body, float PosX, float PosY ); 
	Vertex(){};
};
struct Edge;

class Physics {
	Vec2 Gravity; 

	int BodyCount;
	int VertexCount;
	int EdgeCount;

	Vertex*      Vertices[ MAX_VERTICES ];
	Edge*        Edges   [ MAX_EDGES    ];
	PhysicsBody* Bodies  [ MAX_BODIES   ];

	float Timestep;
	int Iterations;

	void  UpdateForces();
	void  UpdateVerlet();
	void  UpdateEdges ();
	void  IterateCollisions();
	bool  DetectCollision( PhysicsBody* B1, PhysicsBody* B2 );
	void  ProcessCollision();
	float IntervalDistance( float MinA, float MaxA, float MinB, float MaxB );
	bool  BodiesOverlap( PhysicsBody* B1, PhysicsBody* B2 ); 

	struct {
		float Depth;
		Vec2  Normal;

		Edge*   E;
		Vertex* V;
	} CollisionInfo;

public:
	void Update();
	void Render();

	void AddBody  ( PhysicsBody* Body );
	void AddEdge  ( Edge* E );
	void AddVertex( Vertex* V );

	Vertex* FindVertex( int X, int Y );

	Physics( float GravitationX = 0.0f, float GravitationY = 0.0f, int pIterations = 1 ) :
					BodyCount( 0 ), VertexCount( 0 ), EdgeCount( 0 ),
					Gravity( Vec2( GravitationX, GravitationY ) ),
					Iterations( pIterations ), Timestep( 1.0f ) {}
};

struct PhysicsBody {
	Vec2 Center; 

	int MinX, MinY, MaxX, MaxY;

	int VertexCount;
	int EdgeCount;

	Vertex* Vertices[ MAX_BODY_VERTICES ];
	Edge*   Edges   [ MAX_BODY_EDGES    ];

	PhysicsBody(); 

	void AddEdge  ( Edge*   E );
	void AddVertex( Vertex* V );
	void ProjectToAxis( Vec2& Axis, float& Min, float& Max );
	void CalculateCenter(); 

	void CreateBox( int X, int Y, int Width, int Height );
	void CreateCircle(int X, int Y, int r, int n);
	void CreateSoftCircle(int X, int Y, int r, int n);
	void CreateTriangle(int X, int Y);
};

struct Edge {
	Vertex* V1;
	Vertex* V2;
	Vertex* V3;

	float Length;
	int Boundary; 

	PhysicsBody* Parent;

	Edge( PhysicsBody* Body, Vertex* pV1, Vertex* pV2, int pBoundary = true ); 
};

#endif
