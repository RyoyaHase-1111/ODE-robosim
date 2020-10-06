#include<ode\ode.h>
#include<drawstuff\drawstuff.h>

#include <math.h>
#include "StlReader.h"
//

#define ROBOT_NUM 13
#define ROBOT_BODY_W 0.1
#define ROBOT_BODY_D 0.12
#define ROBOT_BODY_H 0.05
#define ROBOT_LEG_R 0.015
#define ROBOT_LEG_L 0.04



//----------------------------------------------------------------------------
double Start_Pos[3] = { 0.0, 0.0, ROBOT_LEG_L + ROBOT_LEG_R };

double Robot_Pos[ROBOT_NUM][3] = { { Start_Pos[0], Start_Pos[1], ROBOT_BODY_H * 0.5 + Start_Pos[2]},
								   { 0.5*ROBOT_BODY_W + (ROBOT_LEG_R + 0.5*ROBOT_LEG_L) + Start_Pos[0], 0.5*ROBOT_BODY_D - ROBOT_LEG_R + Start_Pos[1], ROBOT_LEG_R + Start_Pos[2] }, 
								   { 0.5*ROBOT_BODY_W + (ROBOT_LEG_R + 0.5*ROBOT_LEG_L) + Start_Pos[0], Start_Pos[1], ROBOT_LEG_R + Start_Pos[2] }, 
								   { 0.5*ROBOT_BODY_W + (ROBOT_LEG_R + 0.5*ROBOT_LEG_L) + Start_Pos[0], -0.5*ROBOT_BODY_D + ROBOT_LEG_R + Start_Pos[1], ROBOT_LEG_R + Start_Pos[2] }, 
								   { -0.5*ROBOT_BODY_W - (ROBOT_LEG_R + 0.5*ROBOT_LEG_L) + Start_Pos[0], -0.5*ROBOT_BODY_D + ROBOT_LEG_R + Start_Pos[1], ROBOT_LEG_R + Start_Pos[2] }, 
								   { -0.5*ROBOT_BODY_W - (ROBOT_LEG_R + 0.5*ROBOT_LEG_L) + Start_Pos[0], Start_Pos[1], ROBOT_LEG_R + Start_Pos[2] }, 
								   { -0.5*ROBOT_BODY_W - (ROBOT_LEG_R + 0.5*ROBOT_LEG_L) + Start_Pos[0], 0.5*ROBOT_BODY_D - ROBOT_LEG_R + Start_Pos[1], ROBOT_LEG_R + Start_Pos[2] },
								   { 0.5*ROBOT_BODY_W + (ROBOT_LEG_R*2 + ROBOT_LEG_L) + Start_Pos[0], 0.5*ROBOT_BODY_D - ROBOT_LEG_R + Start_Pos[1], - 0.5*ROBOT_LEG_L + Start_Pos[2] },
								   { 0.5*ROBOT_BODY_W + (ROBOT_LEG_R*2 + ROBOT_LEG_L) + Start_Pos[0], Start_Pos[1], - 0.5*ROBOT_LEG_L + Start_Pos[2] },
								   { 0.5*ROBOT_BODY_W + (ROBOT_LEG_R*2 + ROBOT_LEG_L) + Start_Pos[0], -0.5*ROBOT_BODY_D + ROBOT_LEG_R + Start_Pos[1], - 0.5*ROBOT_LEG_L + Start_Pos[2] },
								   { -0.5*ROBOT_BODY_W - (ROBOT_LEG_R*2 + ROBOT_LEG_L) + Start_Pos[0], -0.5*ROBOT_BODY_D + ROBOT_LEG_R + Start_Pos[1], - 0.5*ROBOT_LEG_L + Start_Pos[2] },
								   { -0.5*ROBOT_BODY_W - (ROBOT_LEG_R*2 + ROBOT_LEG_L) + Start_Pos[0], Start_Pos[1], - 0.5*ROBOT_LEG_L + Start_Pos[2] },
								   { -0.5*ROBOT_BODY_W - (ROBOT_LEG_R*2 + ROBOT_LEG_L) + Start_Pos[0], 0.5*ROBOT_BODY_D - ROBOT_LEG_R + Start_Pos[1], - 0.5*ROBOT_LEG_L + Start_Pos[2] }};
double Robot_Weight[ROBOT_NUM] = { 0.125, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08 };
dBodyID Robot_Body[ROBOT_NUM];
dGeomID Robot_Geom[ROBOT_NUM];
dMass Robot_Mass[ROBOT_NUM];

#define RJOINT_NUM 12
dJointID Rjoint[RJOINT_NUM];
dReal Rjoint_x[RJOINT_NUM] = { 0.5*ROBOT_BODY_W + Start_Pos[0], 0.5*ROBOT_BODY_W + Start_Pos[0], 0.5*ROBOT_BODY_W + Start_Pos[0],
							   -0.5*ROBOT_BODY_W + Start_Pos[0], -0.5*ROBOT_BODY_W + Start_Pos[0], -0.5*ROBOT_BODY_W + Start_Pos[0],
							   0.5*ROBOT_BODY_W + (ROBOT_LEG_R*2 + ROBOT_LEG_L) + Start_Pos[0], 0.5*ROBOT_BODY_W + (ROBOT_LEG_R*2 + ROBOT_LEG_L) + Start_Pos[0], 0.5*ROBOT_BODY_W + (ROBOT_LEG_R*2 + ROBOT_LEG_L) + Start_Pos[0],
							   -0.5*ROBOT_BODY_W - (ROBOT_LEG_R*2 + ROBOT_LEG_L) + Start_Pos[0], -0.5*ROBOT_BODY_W - (ROBOT_LEG_R*2 + ROBOT_LEG_L) + Start_Pos[0], -0.5*ROBOT_BODY_W - (ROBOT_LEG_R*2 + ROBOT_LEG_L) + Start_Pos[0] },
	  Rjoint_y[RJOINT_NUM] = { 0.5*ROBOT_BODY_D - ROBOT_LEG_R + Start_Pos[1], Start_Pos[1], -0.5*ROBOT_BODY_D + ROBOT_LEG_R + Start_Pos[1],
							   -0.5*ROBOT_BODY_D + ROBOT_LEG_R + Start_Pos[1], Start_Pos[1], 0.5*ROBOT_BODY_D - ROBOT_LEG_R + Start_Pos[1],
							   0.5*ROBOT_BODY_D - ROBOT_LEG_R + Start_Pos[1], Start_Pos[1], -0.5*ROBOT_BODY_D + ROBOT_LEG_R + Start_Pos[1],
							   -0.5*ROBOT_BODY_D + ROBOT_LEG_R + Start_Pos[1], Start_Pos[1], 0.5*ROBOT_BODY_D - ROBOT_LEG_R + Start_Pos[1] },
	  Rjoint_z[RJOINT_NUM] = { ROBOT_LEG_R + Start_Pos[2], ROBOT_LEG_R + Start_Pos[2], ROBOT_LEG_R + Start_Pos[2], 
							   ROBOT_LEG_R + Start_Pos[2], ROBOT_LEG_R + Start_Pos[2], ROBOT_LEG_R + Start_Pos[2],
							   ROBOT_LEG_R + Start_Pos[2], ROBOT_LEG_R + Start_Pos[2], ROBOT_LEG_R + Start_Pos[2],
							   ROBOT_LEG_R + Start_Pos[2], ROBOT_LEG_R + Start_Pos[2], ROBOT_LEG_R + Start_Pos[2] };
dReal Joint_Angle[RJOINT_NUM] = { 0.25 * M_PI, 0.0, -0.25 * M_PI, -0.25 * M_PI, 0.0, 0.25 * M_PI, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
dReal Joint_AngVel[RJOINT_NUM] = { 0.0 };
double Move_Time;

bool Robo_view = false,
	 Robo_view2 = false;
//bool Robo_autoCon = false;

////----------------------------------------------------------------------------
//#define OBJECT_NUM 6
//double Obj_Pos[OBJECT_NUM][3] = { { 10.1, 0.0, 1.5 }, { -10.1, 0.0, 1.5 }, { 0.0, 27.6, 1.5 }, { 0.0, -27.6, 1.5 }, 
//								  { 0.0, 12.5, 1.5 }, { 0.0, -12.5, 1.5 }};
//
//double Obj_Size[OBJECT_NUM][3] = { { 0.2, 55.4, 3.0}, { 0.2, 55.4, 3.0}, { 20.0, 0.2, 3.0}, { 20.0, 0.2, 3.0}, { 10.0, 20.0, 3.0}, { 10.0, 20.0, 3.0}};
//double Obj_Weight[OBJECT_NUM] = { 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
//dBodyID Obj_Body[OBJECT_NUM];
//dGeomID Obj_Geom[OBJECT_NUM];
//dMass Obj_Mass[OBJECT_NUM];
//
//#define OJOINT_NUM 6
//dJointID Ojoint[OJOINT_NUM];
//dReal Ojoint_x[OJOINT_NUM] = { 10.1, -10.1, 0, 0, 0, 0}, //右、左、前、後
//Ojoint_y[OJOINT_NUM] = { 0, 0, 27.6, -27.6, 12.5, -12.5},
//Ojoint_z[OJOINT_NUM] = { 0, 0, 0, 0, 0, 0};
//

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

dWorldID world;
dGeomID ground;
dSpaceID space;
dJointGroupID contactgroup;
dsFunctions fn;
//

//------------------------------------------------------------------------------
double rnd()
{
	return(((double)rand() / (double)RAND_MAX));
}

//------------------------------------------------------------------------------
double rnd2()
{
	return((double)rand() / ((double)RAND_MAX + 1.0));
}

//------------------------------------------------------------------------------
double rndn()            /*   normal random number */
{
	return (rnd() + rnd() + rnd() + rnd() + rnd() + rnd() +
		rnd() + rnd() + rnd() + rnd() + rnd() + rnd() - 6.0);
}

void RoboCon(double ang1, double ang2, double ang3, double ang4, double ang5, double ang6, 
	double ang7, double ang8, double ang9, double ang10, double ang11, double ang12, double time) {
	
	Move_Time = time;

	Joint_Angle[0] = ang1 / 180.0;
	Joint_Angle[1] = ang2 / 180.0;
	Joint_Angle[2] = ang3 / 180.0;
	Joint_Angle[3] = ang4 / 180.0;
	Joint_Angle[4] = ang5 / 180.0;
	Joint_Angle[5] = ang6 / 180.0;
	Joint_Angle[6] = ang7 / 180.0;
	Joint_Angle[7] = ang8 / 180.0;
	Joint_Angle[8] = ang9 / 180.0;
	Joint_Angle[9] = ang10 / 180.0;
	Joint_Angle[10] = ang11 / 180.0;
	Joint_Angle[11] = ang12 / 180.0;

	for (int i = 0; i < RJOINT_NUM; i++) Joint_Angle[i] *= M_PI;

}

//------------------------------------------------------------------------------
void control()
{
	dReal d;	//difference from target
	dReal ang;     //current angle

	double fMax = 1.5; //Maxトルク(N m)
	
	for (int i = 0; i < RJOINT_NUM; i++) {
		dJointSetHingeParam(Rjoint[i], dParamLoStop, -(5.0 / 6.0) * M_PI);
		dJointSetHingeParam(Rjoint[i], dParamHiStop, (5.0 / 6.0) * M_PI);
		dJointSetHingeParam(Rjoint[i], dParamFMax, fMax);

		ang = dJointGetHingeAngle(Rjoint[i]);
		if(Move_Time > 0)
			Joint_AngVel[i] = (Joint_Angle[i] - ang) / Move_Time;

		dJointSetHingeParam(Rjoint[i], dParamVel, Joint_AngVel[i]);
	}
}

dJointID collision_joint;
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	int i, j;
	//int c;
	static const int N = 10;
	double pos[3], d;
	dContact contact[N];

	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;

	int isGround = ((ground == o1) || (ground == o2)),
		isRobot = false;

	for (i = 0; i < 7; i++) {
		if ((Robot_Geom[i] == o1) || (Robot_Geom[i] == o2)){
			if((((Robot_Geom[i] == o1) && (Robot_Geom[i + 6] == o2)) || ((Robot_Geom[i] == o2) && (Robot_Geom[i + 6] == o1))) && i > 0)
				isRobot = false;
			else {
				isRobot = true;
				break;
			}
		}
	}

	int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));

	if (n > 0) {
		if (isGround) {
			for (int i = 0; i < n; i++) {
				contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
				contact[i].surface.mu = dInfinity;
				contact[i].surface.slip1 = 0.01;
				contact[i].surface.slip2 = 0.01;
				contact[i].surface.soft_erp = 1.0;
				contact[i].surface.soft_cfm = 1e-4;

				dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
				dJointAttach(c, dGeomGetBody(contact[i].geom.g1),
					dGeomGetBody(contact[i].geom.g2));

			}
		}
		else if (isRobot) {
			for (int i = 0; i < n; i++) {
				contact[i].surface.mode = dContactBounce;
				contact[i].surface.mu = dInfinity;
				contact[i].surface.bounce = 0.0;
				contact[i].surface.bounce_vel = 0.0;

				dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
				dJointAttach(c, dGeomGetBody(contact[i].geom.g1),
					dGeomGetBody(contact[i].geom.g2));

			}
		}
	}
}

//------------------------------------------------------------------
void make_Environment() 
{
	//for (int i = 0; i < OBJECT_NUM; i++) {
	//	Obj_Body[i] = dBodyCreate(world);
	//	dMassSetZero(&Obj_Mass[i]);
	//	dMassSetBoxTotal(&Obj_Mass[i], Obj_Weight[i], Obj_Size[i][0], Obj_Size[i][1], Obj_Size[i][2]);
	//	dBodySetMass(Obj_Body[i], &Obj_Mass[i]);
	//	dBodySetPosition(Obj_Body[i], Obj_Pos[i][0], Obj_Pos[i][1], Obj_Pos[i][2]);

	//	Obj_Geom[i] = dCreateBox(space, Obj_Size[i][0], Obj_Size[i][1], Obj_Size[i][2]);
	//	dGeomSetBody(Obj_Geom[i], Obj_Body[i]);

	//	Ojoint[i] = dJointCreateFixed(world, 0);
	//	dJointAttach(Ojoint[i], Obj_Body[i], 0);
	//	dJointSetFixed(Ojoint[i]);
	//}
}

void drw_Environment() 
{
	//const dReal *pos, *R;
	//double p[3];
	//
	//for (int i = 0; i < OBJECT_NUM; i++) {
	//	for (int j = 0; j < 3; j++)
	//		p[j] = Obj_Size[i][j];
	//	dsSetColorAlpha(1.0, 1.0, 1.0, 1.0);
	//	pos = dBodyGetPosition(Obj_Body[i]);
	//	R = dBodyGetRotation(Obj_Body[i]);
	//	dsDrawBox(pos, R, p);
	//}

	//dsSetColorAlpha(0.3, 1.0, 1.0, 0.6);
	//dsDrawCylinder (Goal_Pos, R, 2.0, 0.5);
}


//------------------------------------------------------------------
void make_Robot()
{
	//ロボット本体
	Robot_Body[0] = dBodyCreate(world);
	dMassSetZero(&Robot_Mass[0]);
	dMassSetBoxTotal(&Robot_Mass[0], Robot_Weight[0], ROBOT_BODY_W, ROBOT_BODY_D, ROBOT_BODY_H);
	dBodySetMass(Robot_Body[0], &Robot_Mass[0]);
	dBodySetPosition(Robot_Body[0], Robot_Pos[0][0], Robot_Pos[0][1], Robot_Pos[0][2]);

	Robot_Geom[0] = dCreateBox(space, ROBOT_BODY_W, ROBOT_BODY_D, ROBOT_BODY_H);
	dGeomSetBody(Robot_Geom[0], Robot_Body[0]);

	dMatrix3 R;
	dRFromAxisAndAngle(R, 0, 1, 0, M_PI*0.5);

	for (int i = 1; i < 7; i++) {
		Robot_Body[i] = dBodyCreate(world);
		dMassSetZero(&Robot_Mass[i]);
		dMassSetCapsuleTotal(&Robot_Mass[i], Robot_Weight[i], 3, ROBOT_LEG_R, ROBOT_LEG_L);
		dBodySetMass(Robot_Body[i], &Robot_Mass[i]);
		dBodySetPosition(Robot_Body[i], Robot_Pos[i][0], Robot_Pos[i][1], Robot_Pos[i][2]);
		dBodySetRotation(Robot_Body[i], R);

		Robot_Geom[i] = dCreateCapsule(space, ROBOT_LEG_R, ROBOT_LEG_L);
		dGeomSetBody(Robot_Geom[i], Robot_Body[i]);
	}

	for (int i = 7; i < ROBOT_NUM; i++) {
		Robot_Body[i] = dBodyCreate(world);
		dMassSetZero(&Robot_Mass[i]);
		dMassSetCapsuleTotal(&Robot_Mass[i], Robot_Weight[i], 3, ROBOT_LEG_R, ROBOT_LEG_L);
		dBodySetMass(Robot_Body[i], &Robot_Mass[i]);
		dBodySetPosition(Robot_Body[i], Robot_Pos[i][0], Robot_Pos[i][1], Robot_Pos[i][2]);

		Robot_Geom[i] = dCreateCapsule(space, ROBOT_LEG_R, ROBOT_LEG_L);
		dGeomSetBody(Robot_Geom[i], Robot_Body[i]);
	}

	for (int i = 0; i < 6; i++) {
		Rjoint[i] = dJointCreateHinge(world, 0);
		dJointAttach(Rjoint[i], Robot_Body[0], Robot_Body[i+1]);
		if(i < 3) dJointSetHingeAxis(Rjoint[i], 0, 0, -1);
		else dJointSetHingeAxis(Rjoint[i], 0, 0, 1);
		dJointSetHingeAnchor(Rjoint[i], Rjoint_x[i], Rjoint_y[i], Rjoint_z[i]);

		Rjoint[i+6] = dJointCreateHinge(world, 0);
		dJointAttach(Rjoint[i + 6], Robot_Body[i + 1], Robot_Body[i + 7]);
		if (i < 3) dJointSetHingeAxis(Rjoint[i + 6], 0, 1, 0);
		else dJointSetHingeAxis(Rjoint[i + 6], 0, -1, 0);
		dJointSetHingeAnchor(Rjoint[i + 6], Rjoint_x[i + 6], Rjoint_y[i + 6], Rjoint_z[i + 6]);
	}
}

void drw_Robot()
{
	const dReal *pos, *R;
	double size[3] = { ROBOT_BODY_W, ROBOT_BODY_D, ROBOT_BODY_H };

	dsSetColorAlpha(0.7, 0.7, 1.0, 1.0);
	pos = dBodyGetPosition(Robot_Body[0]);
	R = dBodyGetRotation(Robot_Body[0]);
	dsDrawBox(pos, R, size);

	for (int i = 1; i < ROBOT_NUM; i++) {
		dsSetColorAlpha(0.7, 0.7, 1.0, 1.0);
		pos = dBodyGetPosition(Robot_Body[i]);
		R = dBodyGetRotation(Robot_Body[i]);
		dsDrawCapsule(pos, R, ROBOT_LEG_L, ROBOT_LEG_R);
	}
}

//------------------------------------------------------------------------------
void start()
{
	static float xyz[3] = { 0.0, -0.2, 0.3 };
	static float hpr[3] = { 90, -45, 0 };
	dsSetViewpoint(xyz, hpr);

}


//------------------------------------------------------------------------------
void command_func(int cmd)
{

	switch (cmd) {
	case '0':
		RoboCon(45.0, 0.0, -45.0, -45.0, 0.0, 45.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
		break;
	case '1':
		RoboCon(0.0, 0.0, -45.0, 0.0, 0.0, 45.0, 40.0, 0.0, 40.0, 0.0, 40.0, 0.0, 1.0);
		break;
	case '2':
		RoboCon(45.0, 0.0, 0.0, -45.0, 0.0, 0.0, 40.0, 0.0, 40.0, 0.0, 40.0, 0.0, 1.0);
		break;
	case '3':
		RoboCon(45.0, 0.0, 0.0, -45.0, 0.0, 0.0, 0.0, 40.0, 0.0, 40.0, 0.0, 40.0, 1.0);
		break;
	case '4':
		RoboCon(0.0, 0.0, -45.0, 0.0, 0.0, 45.0, 0.0, 40.0, 0.0, 40.0, 0.0, 40.0, 1.0);
		break;
	case '5':
		RoboCon(45.0, 0.0, 0.0, 0.0, 0.0, 45.0, 40.0, 0.0, 40.0, 0.0, 40.0, 0.0, 1.0);
		break;
	case '6':
		RoboCon(45.0, 45.0, 0.0, -45.0, 0.0, 0.0, 40.0, 0.0, 40.0, 0.0, 40.0, 0.0, 1.0);		
		break;
	case '7':
		RoboCon(45.0, 45.0, 0.0, -45.0, 0.0, 0.0, 0.0, 40.0, 0.0, 40.0, 0.0, 40.0, 1.0);
		break;
	case '8':
		RoboCon(45.0, 0.0, 0.0, 0.0, 0.0, 45.0, 0.0, 40.0, 0.0, 40.0, 0.0, 40.0, 1.0);
		break;
	case '9':

		break;
	case 32: //スペース押したとき
		if (Robo_view) Robo_view = false;
		else Robo_view = true;
		break;
	}

}

//------------------------------------------------------------------------------
static void simLoop(int pause)
{
	float xyz[3], hpr[3];
	const dReal *pos, *R;
	dReal r11, r12, r13, r21, r22, r23, r31, r32, r33;
	dReal pitch, yaw, roll;

	dSpaceCollide(space, 0, &nearCallback);

	dWorldStep(world, 0.01);
	dJointGroupEmpty(contactgroup);

	drw_Robot();
	//drw_Environment();

	control();

	if (Robo_view) {
		pos = dBodyGetPosition(Robot_Body[0]);
		R = dBodyGetRotation(Robot_Body[0]);
		r11 = R[0]; r12 = R[1]; r13 = R[2];
		r21 = R[4]; r22 = R[5]; r23 = R[6];
		r31 = R[8]; r32 = R[9]; r33 = R[10];
		pitch = atan2(-r31, sqrt(r32*r32 + r33*r33));
		yaw = atan2(r21, r11);
		roll = atan2(r32, r33);
		hpr[2] = 180.0*roll / M_PI; hpr[1] = 180.0*pitch / M_PI; hpr[0] = 90.0+180.0*yaw / M_PI;
		xyz[0] = pos[0] + ROBOT_BODY_D * 4.0 * sin(yaw); xyz[1] = pos[1] - ROBOT_BODY_D * 4.0 * cos(yaw); xyz[2] = pos[2] + ROBOT_BODY_H * 4.0;
	}
	else {
		xyz[0] = 0.0; xyz[1] = -1.0; xyz[2] = 2.0;
		hpr[0] = 90; hpr[1] = -60; hpr[2] = 0;
	};
	dsSetViewpoint(xyz, hpr);

}

//------------------------------------------------------------------------------
void setDrawStuff()
{
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command_func;
	fn.stop = NULL;
	fn.path_to_textures = "drawstuff/textures";
}

//------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	dInitODE();
	setDrawStuff();

	srand((unsigned)time(NULL));

	world = dWorldCreate();
	dWorldSetGravity(world, 0, 0, -9.8);

	space = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);
	ground = dCreatePlane(space, 0, 0, 1, 0);

	make_Robot();
	//make_Environment();
	
	dsSimulationLoop(argc, argv, 500, 500, &fn);
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();
	return 0;
}

//------------------------------------------------------------------------------