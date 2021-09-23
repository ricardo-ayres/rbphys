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
rbphys is a header only simple rigid body physics library based on raymath.
*/

#include <raymath.h>

typedef struct rbp_body {
	/* inverse of mass and inverse of inertia tensor in body space */
	float Minv;
	Matrix Ibinv;

	/* state:
	 * p = linear momentum
	 * L = angular momentum
	 */

	Vector3 pos;
	Vector3 p;
	Quaternion dir;
	Vector3 L;

	Vector3 (*support)(struct rbp_body *self, Vector3 direction);
} rbp_body;

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
rbp_chframe(rbp_body *b, Vector3 v)
{
	return Vector3Add(b->pos, Vector3RotateByQuaternion(v, b->dir));
}

/* Auxiliary variables */
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

Vector3
rbp_displace(rbp_body *b, float dt)
{
	return Vector3Add(b->pos, Vector3Scale(rbp_v(b), dt));
}

Quaternion
rbp_rotate(rbp_body *b, float dt)
{
	Vector3 w3dt = Vector3Scale(rbp_w(b), dt);
	Quaternion rot = QuaternionFromAxisAngle(w3dt, Vector3Length(w3dt));
	return QuaternionNormalize(QuaternionMultiply(rot, b->dir));
}

void
rbp_wspace_force(rbp_body *b, Vector3 force, Vector3 pos, float dt)
{
	Vector3 dp = Vector3Scale(force, dt);
	Vector3 r = Vector3Subtract(pos, b->pos);
	Vector3 dL = Vector3CrossProduct(r, dp);

	b->p = Vector3Add(b->p, dp);
	b->L = Vector3Add(b->L, dL);
	return;
}

void
rbp_bspace_force(rbp_body *b, Vector3 force, Vector3 pos, float dt)
{
	Vector3 wspace_force = Vector3RotateByQuaternion(force, b->dir);
	Vector3 wspace_pos = rbp_chframe(b, pos);
	rbp_wspace_force(b, wspace_force, wspace_pos, dt);
	return;
}

/* GJK Collision detection */
typedef struct rbp_simplex {
	/* simplex dimension */
	int n;

	/* search direction */
	Vector3 dir;

	/* simplex vertices */
	Vector3 A;
	Vector3 B;
	Vector3 C;
	Vector3 D;
} rbp_simplex;

/* some macros to ease typing */
#define NEG(d) Vector3Negate(d)
#define SUB(a, b) Vector3Subtract(a, b);
#define DOT(a, b) Vector3DotProduct(a, b)
#define X(a, b) Vector3CrossProduct(a, b)
#define X3(a, b) Vector3CrossProduct(Vector3CrossProduct(a, b), a)

Vector3
rbp_support(rbp_body *a, rbp_body *b, Vector3 d)
{
	Vector3 sa = a->support(a, d);
	Vector3 sb = b->support(b, NEG(d));
	return Vector3Subtract(sa, sb);
}

int
rbp_u1simplex(rbp_simplex *s)
{
	Vector3 ab = SUB(s->B, s->A);
	Vector3 ao = NEG(s->A);

	if (DOT(ab, ao) > 0) {
		/* update direction and save B */
		s->dir = X3(ab, ao);
		s->C = s->B;
	} else {
		/* update direction and return to 0-simplex */
		s->dir = ao;
		s->n = 0;
	}
	/* save A */
	s->B = s->A;
	return 0;
}

int
rbp_u2simplex(rbp_simplex *s)
{
	Vector3 ab = SUB(s->B, s->A);
	Vector3 ac = SUB(s->C, s->A);
	Vector3 ao = NEG(s->A);
	Vector3 abc = X(ab, ac);

	Vector3 edgenormal = X(abc, ac);
	
	if (DOT(edgenormal, ao) > 0) {
		if (DOT(ac, ao) > 0) {
			/* AC is the simplex */
			s->n = 1;
			s->dir = X3(ac, ao);
		} else {
			/* AB is the simplex, try again */
			s->n = 1;
			return rbp_u1simplex(s);
		}
	} else {
		edgenormal = X(abc, ab);
		if (DOT(edgenormal, ab) > 0) {
			/* AB is the simplex, try again */
			s->n = 1;
			return rbp_u1simplex(s);
		} else if (DOT(abc, ao) > 0) {
			/* continue with abc to next dimension */
			s->D = s->C;
			s->C = s->B;
			s->dir = abc;
		} else {
			/* continue with acb to next dimension */
			s->D = s->B;
			//s->C = s->C; // kinda pointless...
			s->dir = NEG(abc);
		}
	}

	/* save A */
	s->B = s->A;
	return 0;
}

int
rbp_u3simplex(rbp_simplex *s)
{
	Vector3 ab = SUB(s->B, s->A);
	Vector3 ac = SUB(s->C, s->A);
	Vector3 ad = SUB(s->D, s->A);
	Vector3 ao = NEG(s->A);

	Vector3 abc = X(ab, ac);
	Vector3 acd = X(ac, ad);
	Vector3 adb = X(ad, ab);

	if (DOT(abc, ao) > 0) {
		/* origin is above abc face, discard D and try again */
		s->n = 2;
		return rbp_u2simplex(s);
	}

	if (DOT(acd, ao) > 0) {
		s->B = s->C;
		s->C = s->D;
		s->n = 2;
		return rbp_u2simplex(s);
	}

	if (DOT(adb, ao) > 0) {
		s->C = s->B;
		s->B = s->D;
		s->n = 2;
		return rbp_u2simplex(s);
	}

	/* in case all fails, we know we have the origin inside */
	return 1;
}

int
rbp_update_simplex(rbp_simplex *s)
{
	switch(s->n) {
	case 1: return rbp_u1simplex(s);
	case 2: return rbp_u2simplex(s);
	case 3: return rbp_u3simplex(s);
	default: return 0;
	}
}

int
rbp_gjk(rbp_body *b1, rbp_body *b2)
{
	rbp_simplex s;

	/* initial setup */
	s.dir = (Vector3) {1.0f, 0.0f, 0.0f};
	s.B = rbp_support(b1, b2, s.dir);
	s.dir = NEG(s.B);
	s.n = 0;

	while(1) {
		s.A = rbp_support(b1, b2, s.dir);
		s.n++;
		
		/* return early if we can't go past the origin */
		if (DOT(NEG(s.A), s.dir) > 0)
			return 0;

		if (rbp_update_simplex(&s))
			return 1;
	}
}
#undef NEG
#undef SUB
#undef DOT
#undef X
#undef X3
