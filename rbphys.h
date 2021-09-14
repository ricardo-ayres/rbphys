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
	 * L = angular momentum (has to be Vector4 to be compatible with Ibinv and rot)
	 */
	Vector3 pos;
	Vector3 p;
	Matrix rot;
	Vector3 w;
} rbp_body;

/* Auxiliary variables */
Vector3 velocity(rbp_body *b)
{
	return Vector3Scale(b->p, b->Minv);
}

Matrix Iinv(rbp_body *b)
{
	Matrix m1 = MatrixMultiply(b->rot, b->Ibinv);
	return MatrixMultiply(m1, MatrixTranspose(b->rot));
}

/* new orientation */
Matrix drotdt(rbp_body *b)
{
	return MatrixMultiply(Iinv(b), b->rot);
}

