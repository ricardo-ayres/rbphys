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
typedef struct rbp_body {
	/* inverse of mass and inverse of inertia tensor in body space */
	float Minv;
	Matrix Ibinv;

	/* state:
	 * p = linear momentum
	 * L = angular momentum (has to be Vector4 to be compatible with Ibinv and rot)
	 */
	Vector3 pos;
	Vector3 p;
	Quaternion rot;
	Vector4 L;
} rbp_body;

/* Additional math functions */
Vector3 MatrixVectorMultiply(Matrix m, Vector4 v)
{
	Vector4 result;
	result.x = v.x * m11 + v.y * m12 + v.z * m13 + v.w * m14;
	result.y = v.x * m21 + v.y * m22 + v.z * m23 + v.w * m24;
	result.z = v.x * m31 + v.y * m32 + v.z * m33 + v.w * m34;
	result.w = v.x * m41 + v.y * m42 + v.z * m43 + v.w * m44;
}

/* Auxiliary variables */
Vector3 velocity(rbp_body *b)
{
	return Vector3Scale(b->p, b->Minv);
}

Matrix Iinv(rbp_body *b)
{
	Matrix rot = QuaternionToMatrix(QuaternionNormalize(b->rot));
	Matrix m1 = MatrixMultiply(rot, b->Ibinv);

	return MatrixMultiply(m1, MatrixTranspose(rot));
}

Vector4 ang_velocity(rbp_body *b)
{
	/* L = I*w => w = Iinv*L */
	return MatrixVectorMultiply(Iinv(b), b->L);
}

/* new orientation */
Quaternion ddtrot(rbp_body *b)
{
	return QuaternionMultiply(ang_velocity(b), b->rot);
}

