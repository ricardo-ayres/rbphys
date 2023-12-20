/* MPR Collision detection for rbphys */
typedef struct rbp_portal {
	Vector3 V; /* V is V0, deep within the minkowski difference */
	Vector3 Oray; /* Oray is V-O, the ray from V0 to the origin */
	Vector3 A;
	Vector3 B;
	Vector3 C;
	Vector3 N; /* portal normal vector in winding ABC */
} rbp_portal;

/* Defining some macros to ease typing */
#define NEG(d) Vector3Negate(d)
#define SUB(a, b) Vector3Subtract(a, b);
#define DOT(a, b) Vector3DotProduct(a, b)
#define X(a, b) Vector3CrossProduct(a, b)
#define X3(a, b) Vector3CrossProduct(Vector3CrossProduct(a, b), a)
#define NORM(a, b, c) X(SUB(b, a), SUB(c, a))

/* Calls the support-mappings from each body and returns the
 * minkowski difference.
 */
Vector3
rbp_support(rbp_body *a, rbp_body *b, Vector3 d)
{
	Vector3 sa = a->support(a, d);
	Vector3 sb = b->support(b, NEG(d));
	return Vector3Subtract(sa, sb);
}

float rbp_plucker_dot(Vector3 a, Vector3 b)
{
	Vector3 v = SUB(b, a);
	Vector3 pxv = X(a, v);
	return DOT(v, pxv);
}

int
rbp_ray_triangle_test(Vector3 A, Vector3 B, Vector3 C, Vector3 o, Vector3 u)
{
	Vector3 v = X(o, u);
	float ray = DOT(u, v);
	float w1 = ray + rbp_plucker_dot(A, B);
	float w2 = ray + rbp_plucker_dot(B, C);
	float w3 = ray + rbp_plucker_dot(C, A);

	if (w1*w2 > 0 && w1*w3 > 0) {
		/* all windings have the same sign, ray hits triangle */
		return 1;
	}

	/* else, at least one is of different sign, ray doesn't hit triangle */
	return 0;
}

int
rbp_mpr(rbp_body *b1, rbp_body *b2)
{
	rbp_portal p;
	Vector3 dir;
	Vector3 X;

	/* Phase 1: Portal discovery */
	/* Define a line from deep within the minkowski difference and get
	 * a support in the direction pointing to the origin */
	p.V = SUB(b1->pos, b2->pos);
	p.Oray = NEG(p.V); /* V-O, the direction from V to the origin */
	p.A = rbp_support(b1, b2, p.Oray);

	/* Find the direction orthogonal to VA that points to the origin */
	dir = X3(SUB(p.A, p.V), p.Oray);
	p.B = rbp_support(b1, b2, dir);

	/* Now VAB defines a triangle, find the normal that points towards the
	 * origin and get the final support that will define our initial
	 * portal ABC. */
	dir = X(SUB(p.A, p.V), SUB(p.B, p.V));
	if (DOT(dir, p.Oray) < 0) {
		/* origin is on the other side, flip. */
		dir = NEG(dir);
	}
	p.C = rbp_support(b1, b2, dir);
	p.N = NORM(p.A, p.B, p.C);
	/* Initial portal "ABC" is complete */

	/* Phase 2: Portal refinement */
	while (1) {
		/* Check wether the origin is before or beyond the portal. */
		if (DOT(p.N, p.Oray) < 0) {
			/* origin is behind the portal, return a hit */
			return 1;
		}

		/* Origin is beyond the portal, get a new support point in the
		 * direction of the portal normal and see if the origin is beyond
		 * the plane defined by the new support and the portal normal. */
		X = rbp_support(b1, b2, p.N);
		dir = NEG(X) /* ray from X to the origin */
		if (DOT(p.N, dir) > 0) {
			/* origin is beyond the plane, so outside of the minkowski
			 * difference. Return a miss */
			return 0;
		}

		/* Origin is before X, refine the portal: Use X to form 3 new
		 * candidate portals from the remaining faces of the tetrahedron
		 * ABCX: ABX, AXC, and BCX */

		/* test ABX, AXC and BCX using plucker coordinates */
		if (rbp_ray_triangle_test(p.A, p.B, X, p.V, p.Oray) > 0) {
			/* ABX is the new portal, update C and continue */
			p.C = X;
		} else if (rbp_ray_triangle_test(p.A, X, p.C, p.V, p.Oray) > 0) {
			/* AXC is the new portal, update B and continue */
			p.B = X;
		} else if (rbp_ray_triangle_test(p.B, p.C, X, p.V, p.Oray) > 0) {
			/* BCX is the new portal, update ABC and continue */
			p.A = p.B;
			p.B = p.C;
			p.C = X;
		}
	}	
}

/* Cleanup macros */
#undef NEG
#undef SUB
#undef DOT
#undef X
#undef X3
#undef NORM
