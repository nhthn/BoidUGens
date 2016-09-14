#include "SC_PlugIn.h"

// boids oscillator
// 200910 oswald berthold
// GPL

// todo:
//  - use separate output for each boid, so it could be used to
//    drive a graphical system

// boids ugen:
//  - array boids mit allen agenten
//   - pos (x,y,z)
//   - vel (x,y,z)

// calc_a:
// for boid in boids
//  - rule1
//  - rule2
//  - rule3
//  - write-add pos.x und pos.y auf bus 1,2

// defines
#define MAX_BOIDS 20

// InterfaceTable contains pointers to functions in the host (server).
static InterfaceTable *ft;

// vector structure
struct vec {
  double x;
  double y;
  double z;
};

// vector arithmetic
void vec_add(struct vec *v1, struct vec *v2) {
  v1->x = v1->x + v2->x;
  v1->y = v1->y + v2->y;
  v1->z = v1->z + v2->z;
}

void vec_sub(struct vec *v1, struct vec *v2) {
  v1->x = v1->x - v2->x;
  v1->y = v1->y - v2->y;
  v1->z = v1->z - v2->z;
}

void vec_mul_scalar(struct vec *v1, double scalar) {
  v1->x *= scalar;
  v1->y *= scalar;
  v1->z *= scalar;
}

void vec_div_scalar(struct vec *v1, double scalar) {
  scalar = 1.0/scalar;
  v1->x *= scalar;
  v1->y *= scalar;
  v1->z *= scalar;
}

double vec_norm(struct vec *v1) {
  return sqrt(pow(v1->x,2) + pow(v1->y, 2) + pow(v1->z, 2));
}

struct X75Boids : public Unit
{
  int numboids;
  double nbadj; // adjust amplitude to numboids
  double nbm1adj; // adjust amplitude to numboids-1
  struct vec *boidpos[MAX_BOIDS]; // boid container
  struct vec *boidvel[MAX_BOIDS]; // boid container
  double diss;  // dissipative term
  double f1, f2, f3; // force factor for rule1-3
};

extern "C"
{
  void load(InterfaceTable *inTable);
  void X75Boids_next_a(X75Boids *unit, int inNumSamples);
  void X75Boids_Ctor(X75Boids* unit);
  void X75Boids_Dtor(X75Boids* unit);
  void X75Boids_rule1(X75Boids* unit, int bid);
  void X75Boids_rule2(X75Boids* unit, int bid);
  void X75Boids_rule3(X75Boids* unit, int bid);
  void X75Boids_rule4(X75Boids* unit, int bid);
  void X75Boids_rule5(X75Boids* unit, int bid);
};

void X75Boids_Ctor(X75Boids* unit)
{
  int i;

  SETCALC(X75Boids_next_a);

  unit->numboids = IN0(1);
  if((unit->numboids) >= MAX_BOIDS) {
    unit->numboids = MAX_BOIDS;
  }
  unit->diss = IN0(2);
  unit->f1 = IN0(3);
  unit->f2 = IN0(4);
  unit->f3 = IN0(5);
  unit->nbadj = 1.0/unit->numboids;
  unit->nbm1adj = 1.0/(unit->numboids-1);
  srand(time(NULL)); // XXX
  printf("X75Boids_Ctor boid init: number %d, diss: %f\n", unit->numboids, unit->diss);
  for(i=0;i<unit->numboids;i++) {
    //printf("X75Boids_Ctor boid init: %d\n", i);
    // init position
    unit->boidpos[i] = (struct vec *)
      RTAlloc(unit->mWorld, sizeof(struct vec)); // randomize
      unit->boidpos[i]->x = 0.2 * (double)(rand())/RAND_MAX - 0.1;
      unit->boidpos[i]->y = 0.2 * (double)(rand())/RAND_MAX - 0.1;
      unit->boidpos[i]->z = 0.2 * (double)(rand())/RAND_MAX - 0.1;
    // init velocity
      unit->boidvel[i] = (struct vec *)RTAlloc(unit->mWorld,
					     sizeof(struct vec)); // randomize
      unit->boidvel[i]->x = 0.01 * (double)(rand())/RAND_MAX - 0.005;
      unit->boidvel[i]->y = 0.01 * (double)(rand())/RAND_MAX - 0.005;
      unit->boidvel[i]->z = 0.01 * (double)(rand())/RAND_MAX - 0.005;
    //printf("X75Boids_Ctor boid init: %f %f %f\n", unit->boids[i]->x,
    //   unit->boids[i]->y, unit->boids[i]->z);
    }

    X75Boids_next_a(unit, 1);
  }

//////////////////////////////////////////////////////////////////

// calculation function for an audio rate frequency argument
  void X75Boids_next_a(X75Boids *unit, int inNumSamples)
  {
  // get the pointer to the output buffers
    float *xout = OUT(0);
    float *yout = OUT(1);
  // get the pointer to the input buffer
  float *in = IN(0); // use for excitation
  double diss = unit->diss = IN0(2);
  double f1 = unit->f1 = IN0(3);
  double f2 = unit->f2 = IN0(4);
  double f3 = unit->f3 = IN0(5);
  float x, y;
  //float z;

  // update unit states
//   if(diss != unit->diss)
//     unit->diss = diss;
//   if(f1 != unit->f1)
//     unit->f1 = f1;
//   if(f2 != unit->f2)
//     unit->f2 = f2;
//   if(f3 != unit->f3)
//     unit->f3 = f3;

  for (int i=0; i < inNumSamples; ++i) // loop block
  {
    x = 0.0;
    y = 0.0;
      //z = 0.0;
      for(int j=0;j<unit->numboids;j++) { // loop boids
// 	printf("X75Boids_next_a pos: %f %f %f\n",
// 	       unit->boidpos[j]->x, unit->boidpos[j]->y,
// 	       unit->boidpos[j]->z);
// 	printf("X75Boids_next_a vel: %f %f %f\n",
// 	       unit->boidvel[j]->x, unit->boidvel[j]->y,
// 	       unit->boidvel[j]->z);
       if(unit->numboids > 1) {
	  X75Boids_rule1(unit, j); // clumping
	  X75Boids_rule2(unit, j); // avoidance
	  X75Boids_rule3(unit, j); // schooling
	}
	X75Boids_rule4(unit, j); // bound region
	X75Boids_rule5(unit, j); // limit speed
	vec_mul_scalar(unit->boidvel[j], diss);
	vec_add(unit->boidpos[j], unit->boidvel[j]);
	x += unit->boidpos[j]->x;
	y += unit->boidpos[j]->y;
}
      //printf("X75Boids_next_a out x: %f, y: %f\n", x, y);
      // write the output
//       xout[i] = unit->boidpos[0]->x;
//       yout[i] = unit->boidpos[0]->y;
      xout[i] = x * unit->nbadj; /// unit->numboids;
      yout[i] = y * unit->nbadj; //unit->numboids;
    }
  }

// clumping
  void X75Boids_rule1(X75Boids *unit, int bid)
  {
    struct vec v = {0, 0, 0};
    double f1 = unit->f1;
    for(int i;i<unit->numboids;i++){
    if(i!=bid) { // not self
      vec_add(&v, unit->boidpos[i]);
    }
  }
  //vec_div_scalar(&v, unit->numboids-1);
  vec_mul_scalar(&v, unit->nbm1adj);
  vec_sub(&v, unit->boidpos[bid]);
  //vec_div_scalar(&v, 500.0);
  vec_mul_scalar(&v, f1);
  vec_add(unit->boidvel[bid], &v);
  //printf("X75Boids_rule1 x: %f, y: %f\n", v.x, v.y);
}

// avoidance
void X75Boids_rule2(X75Boids *unit, int bid)
{
  struct vec v = {0, 0, 0};
  struct vec p;
  double d = 0.0;
  double f2 = unit->f2;
  for(int i;i<unit->numboids;i++){
    if(i!=bid) { // not self
      p = *(unit->boidpos[bid]);
      vec_sub(&p, unit->boidpos[i]);
      d = vec_norm(&p);
      if (d < 0.3)
       vec_add(&v, &p);
   }
 }
  //vec_div_scalar(&v, 500.0);
 vec_mul_scalar(&v, f2);
 vec_add(unit->boidvel[bid], &v);
}

// schooling
void X75Boids_rule3(X75Boids *unit, int bid)
{
  struct vec v = {0, 0, 0};
  double f3 = unit->f3;
  for(int i;i<unit->numboids;i++){
    if(i!=bid) { // not self
      vec_add(&v, unit->boidvel[i]);
    }
  }
  vec_div_scalar(&v, unit->numboids-1);
  //vec_div_scalar(&v, 200.0);
  vec_mul_scalar(&v, f3);
  vec_add(unit->boidvel[bid], &v);
}

// bound region
void X75Boids_rule4(X75Boids *unit, int bid)
{
  float xmin = -0.5;
  float xmax =  0.5;
  float ymin = -0.5;
  float ymax =  0.5;
  float zmin = -0.5;
  float zmax =  0.5;
  struct vec v = {0, 0, 0};
  if(unit->boidpos[bid]->x < xmin)
    v.x = 7e-3;
  else if(unit->boidpos[bid]->x > xmax)
    v.x = -7e-3;
  if(unit->boidpos[bid]->y < ymin)
    v.y = 7e-3;
  else if(unit->boidpos[bid]->y > ymax)
    v.y = -7e-3;
  if(unit->boidpos[bid]->z < zmin)
    v.z = 7e-3;
  else if(unit->boidpos[bid]->z > zmax)
    v.z = -7e-3;
  vec_add(unit->boidvel[bid], &v);
}

// speed limit
void X75Boids_rule5(X75Boids *unit, int bid)
{
  double vlim = 0.2;
  double d = vec_norm(unit->boidvel[bid]);
  if(d > vlim)
    vec_div_scalar(unit->boidvel[bid], d);
}

//////////////////////////////////////////////////
// Dtor is called to perform any clean up for the unit generator. 

void X75Boids_Dtor(X75Boids* unit)
{
  // free the buffer
  int i;
  for(i=0;i<unit->numboids;i++) {
    RTFree(unit->mWorld, unit->boidpos[i]);
    RTFree(unit->mWorld, unit->boidvel[i]);
  }
  //RTFree(unit->mWorld, unit->mData);
}


////////////////////////////////////////////////////////////////////

// the load function is called by the host when the plug-in is loaded
PluginLoad(SWUGens) {
  ft = inTable;
  DefineSimpleUnit(X75Boids);
}

////////////////////////////////////////////////////////////////////
