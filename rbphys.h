/*
MIT/X Consortium License

© 2021-2021 Ricardo B. Ayres <ricardo.bosqueiro.ayres@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

/*
rbphys is a simple rigid body physics library based on raymath.
*/

#include <raymath.h>

/* Shorthands for raymath functions */
#define NEG(a) Vector3Negate(a)
#define DOT(a, b) Vector3DotProduct(a, b)
#define X(a, b) Vector3CrossProduct(a, b)

/* Collider data types */
typedef enum {
	HEIGHTMAP = 0,
	SPHERE,
	CUBOID,
} rbp_collider_type;

/*  This is the 'parent' struct that should be 'inherited' by all collider
 * types.
 * collider_type = shape of the collider;
 * offset = position of the collider relative to body position;
 * e = partial coefficient of restitution;
 * uf_s = coefficient of friction (static);
 * uf_d = coefficient of friction (dynamic);
 * df = damping factor for collisions with friction;
 */
typedef struct rbp_collider {
#define \
	RBP_COLLIDER_PROPS \
	rbp_collider_type collider_type; \
	Vector3 offset; \
	float e; \
	float uf_s; \
	float uf_d; \
	float df;

	RBP_COLLIDER_PROPS
} rbp_collider;

typedef struct rbp_collider_sphere {
	RBP_COLLIDER_PROPS /* inherit from rbp_collider */

	float radius;
} rbp_collider_sphere;

typedef struct rbp_collider_cuboid {
	RBP_COLLIDER_PROPS /* inherit from rbp_collider */

	Quaternion dir;
	float xsize;
	float ysize;
	float zsize;
} rbp_collider_cuboid;

typedef struct rbp_collider_heightmap {
	RBP_COLLIDER_PROPS /* inherit from rbp_collider */
} rbp_collider_heightmap;

/* Body data type */
typedef struct rbp_body {
	/* inverse of mass and inverse of inertia tensor in body space */
	float Minv;
	Matrix Ibinv;

	/* state:
	 * pos = position in world space
	 * p = linear momentum
	 * dir = orientation quaternion
	 * L = angular momentum
	 */

	Vector3 pos;
	Vector3 p;
	Quaternion dir;
	Vector3 L;

	/* Function pointer to support-mapping for collision detection
	 * Attention: must return support points in world space! */
	/* Vector3 (*support)(struct rbp_body *self, Vector3 direction); */

	/* Pointer to a body collider */
	void *collider;
} rbp_body;

/* Ditching gjk and mpr in favor of a simple analytical collision system */
/* #include "rbp-gjk.h" */
/* #include "rbp-mpr.h" */

/* Collision contact data type */
typedef struct rbp_contact {
	
	/* b1 and b2 = bodies involved in the collision
	 * p1 = contact point for b1 in world space
	 * p2 = contact point for b2 in world space
	 * cn = collision normal (b1->b2, normalized)
	 * depth = penetration depth
	 * e = coefficient of restitution of the collision
	 * uf_s = coefficient of friction (static)
	 * uf_d = coefficient of friction (dynamic)
	 * df1 = damping factor for friction collisions, b1
	 * df2 = damping factor for friction collisions, b2
	 */
	rbp_body *b1;
	rbp_body *b2;
	Vector3 p1;
	Vector3 p2;
	Vector3 cn;
	float depth;
	float e;
	float uf_s;
	float uf_d;
	float df1;
	float df2;
} rbp_contact;

/* Additional math functions */
Vector4
MatrixVectorMultiply(Matrix m, Vector4 v)
{
	Vector4 result;
	result.x = v.x * m.m0 + v.y * m.m4 + v.z * m.m8 + v.w * m.m12;
	result.y = v.x * m.m1 + v.y * m.m5 + v.z * m.m9 + v.w * m.m13;
	result.z = v.x * m.m2 + v.y * m.m6 + v.z * m.m10 + v.w * m.m14;
	result.w = v.x * m.m3 + v.y * m.m7 + v.z * m.m11 + v.w * m.m15;

	return result;
}

Vector3
MatrixVector3Multiply(Matrix m, Vector3 v)
{
	Vector4 v4 = (Vector4) {v.x, v.y, v.z, 0.0f};
	v4 = MatrixVectorMultiply(m, v4);
	Vector3 result = (Vector3) {v4.x, v4.y, v4.z};
	return result;
}

/* Transform vector v from world space to body space */
Vector3
rbp_wtobspace(rbp_body *b, Vector3 v)
{
	return Vector3Add(b->pos, Vector3RotateByQuaternion(v, b->dir));
}

/* Auxiliary variables */
float
rbp_m(rbp_body *b)
{
	return b->Minv != 0 ? 1.0f / b->Minv : 0.0f;
}

Vector3
rbp_v(rbp_body *b)
{
	return Vector3Scale(b->p, b->Minv);
}

Matrix
rbp_Iinv(rbp_body *b)
{
	Matrix R = QuaternionToMatrix(b->dir);
	Matrix m1 = MatrixMultiply(R, b->Ibinv);
	return MatrixMultiply(m1, MatrixTranspose(R));
}

Vector3
rbp_w(rbp_body *b)
{
	Vector4 L = (Vector4) {b->L.x, b->L.y, b->L.z, 0.0f};
	Vector4 r = MatrixVectorMultiply(rbp_Iinv(b), L);
	return (Vector3) {r.x, r.y, r.z};
}

/* Movement functions */
/* Returns the new position of body b after integrating by dt */
Vector3
rbp_displace(rbp_body *b, float dt)
{
	return Vector3Add(b->pos, Vector3Scale(rbp_v(b), dt));
}

/* Returns the new orientation of body b after integrating by dt */
Quaternion
rbp_rotate(rbp_body *b, float dt)
{
	Vector3 w3dt = Vector3Scale(rbp_w(b), dt);
	Quaternion rot = QuaternionFromAxisAngle(w3dt, Vector3Length(w3dt));
	return QuaternionNormalize(QuaternionMultiply(rot, b->dir));
}

/* Simple shortcut to call both functions above and update the body */
void
rbp_update(rbp_body *b, float dt)
{
	b->pos = rbp_displace(b, dt);
	b->dir = rbp_rotate(b, dt);
}

void
rbp_rewind(rbp_body *b, float dt)
{
	b->pos = rbp_displace(b, -1.0f*dt);
	b->dir = rbp_rotate(b, -1.0f*dt);
}

/* Force application functions */
/* Applies an impulse equivalent to the desired force at world space
 * coordinate pos to body b for duration of dt */
void
rbp_wspace_force(rbp_body *b, Vector3 force, Vector3 pos, float dt)
{
	Vector3 dp = Vector3Scale(force, dt);
	Vector3 r = Vector3Subtract(pos, b->pos);
	Vector3 dL = X(r, dp);

	b->p = Vector3Add(b->p, dp);
	b->L = Vector3Add(b->L, dL);
}

/* Applies an impulse equivalent to the desired force at body space
 * coordinate pos to body b for duration of dt */
void
rbp_bspace_force(rbp_body *b, Vector3 force, Vector3 pos, float dt)
{
	Vector3 wspace_force = Vector3RotateByQuaternion(force, b->dir);
	Vector3 wspace_pos = rbp_wtobspace(b, pos);
	rbp_wspace_force(b, wspace_force, wspace_pos, dt);
}

/* Collision functions */
int
rbp_collide_sphere_sphere(rbp_body *b1, rbp_body *b2, rbp_contact *c)
{
	rbp_collider_sphere *c1 = b1->collider;
	rbp_collider_sphere *c2 = b2->collider;
	float r1 = c1->radius;
	float r2 = c2->radius;
	Vector3 pos1 = Vector3Add(b1->pos, c1->offset);
	Vector3 pos2 = Vector3Add(b2->pos, c2->offset);

	Vector3 cn = Vector3Subtract(pos2, pos1); /* vector from center to center */
	float center_distance = Vector3Length(cn);
	float total_radius = r1 + r2;
	float depth = total_radius - center_distance;

	if (depth > 0) {
		/* Hit! Calculate contact, update c and return 1 */
		cn = Vector3Normalize(cn);
		c->cn = cn;
		c->b1 = b1;
		c->b2 = b2;
		c->depth = depth;
		c->p1 = Vector3Add(pos1, Vector3Scale(cn, r1));
		c->p2 = Vector3Add(c->p1, Vector3Scale(cn, depth));
		c->e= c1->e * c2->e;
		c->uf_s = c1->uf_s + c2->uf_s;
		c->uf_d = c1->uf_d + c2->uf_d;
		c->df1 = c1->df;
		c->df2 = c2->df;
		return 1;
	}
	/* Miss, return 0, don't touch c. */
	return 0;
}

int
rbp_collide_sphere_cuboid(rbp_body *b1, rbp_body *b2, rbp_contact *c)
{
	rbp_collider_sphere *c1 = b1->collider;
	rbp_collider_cuboid *c2 = b2->collider;

	Vector3 pos1 = Vector3Add(b1->pos, c1->offset);
	float radius = c1->radius;

	Vector3 pos2 = Vector3Add(b2->pos, c2->offset);
	Quaternion dir2 = QuaternionMultiply(c2->dir, b2->dir);
	dir2 = QuaternionNormalize(dir2);
	float xsize = c2->xsize * 0.5;
	float ysize = c2->ysize * 0.5;
	float zsize = c2->zsize * 0.5;

	/* Calculate relative position */
	Vector3 r12 = Vector3Subtract(pos1, pos2);
	Quaternion unrot = QuaternionInvert(dir2);
	r12 = Vector3RotateByQuaternion(r12, unrot);

	float dx = fabsf(r12.x) - xsize;
	float dy = fabsf(r12.y) - ysize;
	float dz = fabsf(r12.z) - zsize;

	if (dx < radius && dy < radius && dz < radius) {
		/* Hit! Construct c and return 1 */
		/* Pass the bodies swapped because our normal points from the
		 * cuboid (b2) to the sphere (b1) */
		c->b1 = b2;
		c->b2 = b1;
		c->df1 = c2->df; /* swap df's!, see above */
		c->df2 = c1->df;
		c->e = c1->e * c2->e;
		c->uf_s = c1->uf_s + c2->uf_s;
		c->uf_d = c1->uf_d + c2->uf_d;

		/* Build the contact normal and points in local space */
		c->p1.x = r12.x >= 0 ? fminf(zsize, r12.x) : fmaxf(-1.0f*xsize, r12.x);
		c->p1.y = r12.y >= 0 ? fminf(ysize, r12.y) : fmaxf(-1.0f*ysize, r12.y);
		c->p1.z = r12.z >= 0 ? fminf(zsize, r12.z) : fmaxf(-1.0f*zsize, r12.z);

		c->cn = Vector3Subtract(r12, c->p1);
		c->depth = radius - Vector3Length(c->cn);

		if (c->depth < 0) {
			/* abort on false positives on corner collisions */
			return 0;
		}

		/* Send the contact normal and points to world space */
		c->cn = Vector3RotateByQuaternion(c->cn, dir2);
		c->cn = Vector3Normalize(c->cn);
		c->p1 = Vector3RotateByQuaternion(c->p1, dir2);
		c->p1 = Vector3Add(c->b1->pos, c->p1);
		c->p2 = Vector3Add(c->b2->pos, Vector3Scale(c->cn, -1.0f*radius));

		return 1;
	}

	return 0;
}

int
rbp_collide_cuboid_cuboid(rbp_body *b1, rbp_body *b2, rbp_contact *c)
{
	/* IMPLEMENT */
	return 0;
}

/* shapes vs heigthmap collisions */
int
rbp_collide_sphere_heightmap()
{
	/* IMPLEMENT */
	return 0;
}

int
rbp_collide_cuboid_heightmap()
{
	/* IMPLEMENT */
	return 0;
}

int
rbp_collide(rbp_body *b1, rbp_body *b2, rbp_contact *c)
{
	rbp_body *a;
	rbp_body *b;
	int (*collide)(rbp_body*, rbp_body*, rbp_contact*);

	rbp_collider *ca = (rbp_collider *) b1->collider;
	rbp_collider *cb = (rbp_collider *) b2->collider;
	rbp_collider_type a_type = ca->collider_type;
	rbp_collider_type b_type = cb->collider_type;

	/* Ensure smallest collider_type goes in as b1 */
	a = b1;
	b = b2;
	if (a_type > b_type) {
		a = b2;
		b = b1;

		rbp_collider_type tmp = a_type;
		a_type = b_type;
		b_type = tmp;
	}

	switch (a_type) {
	case SPHERE:
		switch (b_type) {
		case SPHERE: /* sphere vs sphere */
			collide = rbp_collide_sphere_sphere;
			break;
		case CUBOID: /* sphere vs cuboid */
			collide = rbp_collide_sphere_cuboid;
			break;

		case HEIGHTMAP: /* sphere vs heightmap */
			collide = rbp_collide_sphere_heightmap;
			break;
		default: return 0;
		}
		break;

	case CUBOID:
		switch (b_type) {
		case CUBOID: /* cuboid vs cuboid */
			collide = rbp_collide_cuboid_cuboid;
			break;

		case HEIGHTMAP: /* cuboid vs heightmap */
			collide = rbp_collide_cuboid_heightmap;
			break;
		default: return 0;
		}
		break;

	default: return 0;
	}

	return collide(a, b, c);
}

/* Collision resolution */
void
rbp_resolve_collision(rbp_contact *c, float dt)
{
	/* unpack c */
	rbp_body *b1 = c->b1;
	rbp_body *b2 = c->b2;
	Vector3 p1 = c->p1;
	Vector3 p2 = c->p2;
	Vector3 cn = c->cn;
	float depth = c->depth;
	float e = c->e;
	float uf_s = c->uf_s;
	float uf_d = c->uf_d;
	float df1 = c->df1;
	float df2 = c->df2;

	/* Calculate vr */
	Vector3 r1 = Vector3Subtract(p1, b1->pos);
	Vector3 r2 = Vector3Subtract(p2, b2->pos);
	Vector3 vp1 = Vector3Add(rbp_v(b1), X(rbp_w(b1), r1));
	Vector3 vp2 = Vector3Add(rbp_v(b2), X(rbp_w(b2), r2));
	Vector3 vr = Vector3Subtract(vp2, vp1);

	/* Calculate jr */
	float minv = b1->Minv + b2->Minv;
	Vector3 vr_rot1 = MatrixVector3Multiply(b1->Ibinv, X(X(r1, cn),r1)); 
	Vector3 vr_rot2 = MatrixVector3Multiply(b2->Ibinv, X(X(r2, cn),r2));
	Vector3 vr_rot = Vector3Add(vr_rot1, vr_rot2);
	float jr_bot = minv + DOT(vr_rot, cn);
	float jr_top = DOT(Vector3Scale(vr, -(1.0f+e)), cn);
	float jr = jr_top / jr_bot;

	/* Calculate impulses */
	Vector3 dp1 = Vector3Scale(cn, -1.0f*jr);
	Vector3 dp2 = Vector3Scale(cn, +1.0f*jr);
	Vector3 dL1 = Vector3Scale(X(r1, cn), -1.0f*jr);
	Vector3 dL2 = Vector3Scale(X(r2, cn), +1.0f*jr);

	/* Calculate friction impulses */
	Vector3 vn = Vector3Scale(cn, DOT(vr, cn));
	Vector3 vt = Vector3Subtract(vr, vn);
	Vector3 tg;

	static const float tolerance = 0.01f;
	if (Vector3Length(vt) < tolerance) {
		tg = Vector3Zero();
	} else {
		tg = Vector3Normalize(vt);

		float js = uf_s * jr;
		float jd = uf_d * jr;
		/*
		float mtot = rbp_m(b1) + rbp_m(b2);
		float pr = DOT(Vector3Scale(vr, mtot), tg);
		*/
		float pr = DOT(Vector3Scale(vr, 1.0f/minv), tg);

		Vector3 jf1;
		if(pr <= js) {
			jf1 = Vector3Scale(tg, pr);
		} else {
			jf1 = Vector3Scale(tg, jd);
		}

		Vector3 jf2 = NEG(jf1);
		/* Apply friction impulses */
		b1->p = Vector3Add(b1->p, jf1);
		b1->L = Vector3Add(b1->L, X(r1, jf1));
		b2->p = Vector3Add(b2->p, jf2);
		b2->L = Vector3Add(b2->L, X(r2, jf2));

		/* Apply damping impulses */
		b1->p = Vector3Scale(b1->p, df1);
		b1->L = Vector3Scale(b1->L, df1);
		b2->p = Vector3Scale(b2->p, df2);
		b2->L = Vector3Scale(b2->L, df2);
	}
	

	/* Apply collision impulses */
	b1->p = Vector3Add(b1->p, dp1);
	b1->L = Vector3Add(b1->L, dL1);
	b2->p = Vector3Add(b2->p, dp2);
	b2->L = Vector3Add(b2->L, dL2);

	/* Adjust positions to eliminate penetration */
	Vector3 ds1 = Vector3Scale(cn, depth*b1->Minv / minv);
	Vector3 ds2 = Vector3Scale(cn, depth*b2->Minv / minv);
	b1->pos = Vector3Subtract(b1->pos, ds1);
	b2->pos = Vector3Add(b2->pos, ds2);
}

#undef NEG
#undef DOT
#undef X