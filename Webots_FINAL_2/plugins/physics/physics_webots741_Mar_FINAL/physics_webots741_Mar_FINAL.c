#include <ode/ode.h>
#include <plugins/physics.h>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
//
dBodyID body = NULL;
dBodyID spring1 = NULL;
dBodyID spring2 = NULL;
dBodyID spring3 = NULL;
dBodyID spring4 = NULL;
//
FILE *sp1;					/* declare the file pointer, spring1 */
FILE *sp2; 					/* declare the file pointer, spring2 */
FILE *sp3;					/* declare the file pointer, spring3 */
FILE *sp4;					/* declare the file pointer, spring4 */
char line[10];
float spr1=0.0;
float spr2=0.0;
float spr3=0.0;
float spr4=0.0;
//
size_t len = 0;
ssize_t read;
//
int i=0;
int apply_force_for=4;		//in ms
dReal xForce =-1101;		// in Newtons
//int xForce =0;		// in Newtons
dReal preload1 = -0;		// in Newtons
//dReal preload1 = 26.756775;		// in Newtons
//dReal preload1 =0;			// in Newtons
//float tolerance = 0.0005; //0.001;
float tolerance = 0.001;	//0.001;
//
int start=0;
float* data;
int size;
//
// Webots R2025a port: webots_physics_init() takes no arguments any more
// (Webots 7 passed dWorldID world, dSpaceID space, dJointGroupID contactJointGroup; none was used).
void webots_physics_init() {
  // find the body on which you want to apply a force
  body = dWebotsGetBodyFromDEF("QUAD_ROBOT");
  spring1=dWebotsGetBodyFromDEF("SPRING1SOLID");
  spring2=dWebotsGetBodyFromDEF("SPRING2SOLID");  
  spring3=dWebotsGetBodyFromDEF("SPRING3SOLID");
  spring4=dWebotsGetBodyFromDEF("SPRING4SOLID");
}
//
void webots_physics_step() {
  //
  dVector3 f;
  //
  const dReal *rot = dBodyGetAngularVel( body );
  const dReal *quat_ptr;
  double quat[4];
  float quat_len; 
  dBodySetFiniteRotationMode(body,1);
  //
  quat_ptr = dBodyGetQuaternion( body );
  quat[0] =  (float) quat_ptr[0];
  quat[1] =  (float) 0.0;
  quat[2] =  (float) 0.0;
  //quat[3] =  (float) 0.0;
  quat[3] =  (float) quat_ptr[3]; 
  quat_len = (float) sqrt( quat[0] * quat[0] + quat[3] * quat[3] );
  quat[0] = (float) quat[0]/quat_len;
  quat[1] = (float) 0.0;
  quat[2] = (float) 0.0;
  //quat[3] = (float) 0.0;
  quat[3] /= (float) quat_len;
  //
  dBodySetQuaternion( body, quat );
  dBodySetAngularVel( body, 0.0, 0.0, (float) rot[2]);
  //
  const double *BodyLinearVel = dBodyGetLinearVel(body);
  const double *BodyRotation=dBodyGetRotation(body);
  //
  double NextCode[1]={1234};
  double vBody[2] = {0,0};
  double BodyRotationMatrix[3][3] = {{0,0,0},
                                     {0,0,0},
                                     {0,0,0}};
   if ((i<(apply_force_for+start)) & (i>=start)) {
     f[0] = xForce;
     f[1] = 0;
     f[2] = 0;
     // at every time step, add a force to the body
     dBodyAddRelForce(body, f[0], f[1], f[2]);
     dWebotsConsolePrintf("xforce %f \n", xForce);

   }
   /**** Get Spring compressions from Webots ****/
   /*
   sp1 = fopen("M:\\Drive\\PROJECTS\\the_project_(ntua)\\TESTS\\plugins\\physics\\spring1.txt", "r");  // open the file for reading 
   fgets( line, 10, sp1 );
   spr1= atof(line);
   fclose(sp1);  // close the file prior to exiting the routine 
   //
   sp2 = fopen("M:\\Drive\\PROJECTS\\the_project_(ntua)\\TESTS\\plugins\\physics\\spring2.txt", "r");  // open the file for reading 
   fgets( line, 10, sp2 );
   spr2= atof(line);
   fclose(sp2);  // close the file prior to exiting the routine 
   //
   sp3 = fopen("M:\\Drive\\PROJECTS\\the_project_(ntua)\\TESTS\\plugins\\physics\\spring3.txt", "r");  // open the file for reading 
   fgets( line, 10, sp3 );
   spr3= atof(line);
   fclose(sp3);  // close the file prior to exiting the routine 
   //
   sp4 = fopen("M:\\Drive\\PROJECTS\\the_project_(ntua)\\TESTS\\plugins\\physics\\spring4.txt", "r");  // open the file for reading 
   fgets( line, 10, sp4 );
   spr4= atof(line);
   fclose(sp4);  // close the file prior to exiting the routine 
   //
   //dWebotsConsolePrintf("The line is %s \n", line);
   //dWebotsConsolePrintf("Spring1: is %f \n", spr1);
   //
   if (spr1>=tolerance){
     dBodyAddRelForce(spring1, 0, preload1, 0);
     //dWebotsConsolePrintf("on ground 1 %f \n", preload1);
   }
   if (spr2>=tolerance){
     dBodyAddRelForce(spring2, 0, preload1, 0);
     //dWebotsConsolePrintf("on ground 2\n");

   }
   if (spr3>=tolerance){
     dBodyAddRelForce(spring3, 0, preload1, 0);
     //dWebotsConsolePrintf("on ground 3\n");

   }
   if (spr4>=tolerance){
     dBodyAddRelForce(spring4, 0, preload1, 0);
     //dWebotsConsolePrintf("on ground 4\n");

   };
   */  
   //
   i++;
   //
   vBody[0] = -BodyLinearVel[0];
   vBody[1] = -BodyLinearVel[1];
   //dWebotsConsolePrintf("Wxd: %f\n",vBody[0]);
   //omegaBody[0] = -BodyAngularVel[2];
   //
   BodyRotationMatrix[0][0]=BodyRotation[0];
   BodyRotationMatrix[0][1]=BodyRotation[1];
   BodyRotationMatrix[0][2]=BodyRotation[2];
   //
   BodyRotationMatrix[1][0]=BodyRotation[3];
   BodyRotationMatrix[1][1]=BodyRotation[4];
   BodyRotationMatrix[1][2]=BodyRotation[5];
   //
   BodyRotationMatrix[2][0]=BodyRotation[6];
   BodyRotationMatrix[2][1]=BodyRotation[7];
   BodyRotationMatrix[2][2]=BodyRotation[8];
   //
   dWebotsSend(0, vBody, sizeof(vBody));
   dWebotsSend(0, NextCode, sizeof(NextCode));
   dWebotsSend(0, BodyRotationMatrix, sizeof(BodyRotationMatrix));
}

void webots_physics_cleanup() {
  /*
   * Here you need to free any memory you allocated in above, close files, etc.
   * You do not need to free any ODE object, they will be freed by Webots.
   */
}