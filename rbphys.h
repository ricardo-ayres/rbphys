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

	/* Function pointer to support-mapping for collision detection */
	Vector3 (*support)(struct rbp_body *self, Vector3 direction);

} rbp_body;

/* Collision contact data type */
typedef struct rbp_contact {
	
	/* b1 and b2 = bodies involved in the collision
	 * p1 = contact point for b1 in world space
	 * p2 = contact point for b2 in world space
	 * cn = contact normal
	 * depth = penetration depth
	 */

	rbp_body *b1;
	rbp_body *b2;
	Vector3 p1;
	Vector3 p2;
	Vector3 cn;
	float depth;
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

/* Transform vector v from world space to body space */
Vector3
rbp_wtobspace(rbp_body *b, Vector3 v)
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
void rbp_update(rbp_body *b, float dt) {
	b->pos = rbp_displace(b, dt);
	b->dir = rbp_rotate(b, dt);
}

/* Force application functions */
/* Applies force at world space coordinate pos to body b for duration of dt */
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

/* Applies force at body space coordinate pos to body b for duration of dt */
void
rbp_bspace_force(rbp_body *b, Vector3 force, Vector3 pos, float dt)
{
	Vector3 wspace_force = Vector3RotateByQuaternion(force, b->dir);
	Vector3 wspace_pos = rbp_wtobspace(b, pos);
	rbp_wspace_force(b, wspace_force, wspace_pos, dt);
	return;
}

