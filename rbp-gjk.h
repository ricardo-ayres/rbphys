/* GJK Collision detection for rbphys */
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

/* Defining some macros to ease typing */
#define NEG(d) Vector3Negate(d)
#define SUB(a, b) Vector3Subtract(a, b);
#define DOT(a, b) Vector3DotProduct(a, b)
#define X(a, b) Vector3CrossProduct(a, b)
#define X3(a, b) Vector3CrossProduct(Vector3CrossProduct(a, b), a)

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

/* Checks at which side of the line (1-simplex) the origin resides. */
int
rbp_u1simplex(rbp_simplex *s)
{
	Vector3 ab = SUB(s->B, s->A);
	Vector3 ao = NEG(s->A);

	/* Check if ab is in the direction of the origin */
	if (DOT(ab, ao) > 0) {
		/* If yes, save B in the simplex and select orthogonal direction
		 * to search for another point for the 2-simplex */
		s->C = s->B;
		s->dir = X3(ab, ao);
	} else {
		/* Reset direction and return to 0-simplex, ab points in the wrong
		 * direction. */
		s->dir = ao;
		s->n = 0;
	}
	/* save A to B to make space for the new point in the 2-simplex */
	s->B = s->A;
	return 0;
}

/* Checks at which side of the triangle (2-simplex) the origin resides */
int
rbp_u2simplex(rbp_simplex *s)
{
	Vector3 ab = SUB(s->B, s->A);
	Vector3 ac = SUB(s->C, s->A);
	Vector3 ao = NEG(s->A);
	Vector3 abc = X(ab, ac); /* plane normal */

	/* Vector perperndicular to ac and coplanar to the triangle (2-simplex) */
	Vector3 edgenormal = X(abc, ac);

	/* Check if one of the edges (AC or AB) is the feature closest to the
	 * origin. If that is the case, reset to that segment (1-simplex) and
	 * search for a new point to buid the triange (2-simplex).
	 */
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

		/* The current (ABC) triangle is the closest known simplex to the
		 * origin. Check on which side we should search for the next point
		 * to build a tetrahedron (3-simplex). */
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

/* Check if we have the best tetrahedron possible (3-simplex) and if yes,
 * test if the origin is inside. */
int
rbp_u3simplex(rbp_simplex *s)
{
	/* edges */
	Vector3 ab = SUB(s->B, s->A);
	Vector3 ac = SUB(s->C, s->A);
	Vector3 ad = SUB(s->D, s->A);
	Vector3 ao = NEG(s->A);

	/* face normals */
	Vector3 abc = X(ab, ac);
	Vector3 acd = X(ac, ad);
	Vector3 adb = X(ad, ab);

	/* Check if any of the faces are better simplices than the current. */
	if (DOT(abc, ao) > 0) {
		/* origin is above abc face, discard D and try again */
		s->n = 2;
		return rbp_u2simplex(s);
	}

	if (DOT(acd, ao) > 0) {
		/* origin is above acd face, discard B and try again */
		s->B = s->C;
		s->C = s->D;
		s->n = 2;
		return rbp_u2simplex(s);
	}

	if (DOT(adb, ao) > 0) {
		/* origin is above adb face, discard C and try again */
		s->C = s->B;
		s->B = s->D;
		s->n = 2;
		return rbp_u2simplex(s);
	}

	/* If all of the above fails, we know that the origin is below all 4
	 * faces, thus, it is inside the current tetrahedron (3-simplex).
	 * Return a hit. */
	return 1;
}

int
rbp_update_simplex(rbp_simplex *s)
{
	/* Select which function to call based on the current simplex dimension.
	 * Cases 1, and 2 are guaranteed to return 0. Only case 3 can report
	 * a hit and return 1. */
	switch(s->n) {
	case 1: return rbp_u1simplex(s); /* Always returns 0 */
	case 2: return rbp_u2simplex(s); /* Always returns 0 */
	case 3: return rbp_u3simplex(s); /* Can return 0 or 1 */
	default: return 0;
	}
}

int
rbp_gjk(rbp_body *b1, rbp_body *b2, Vector3 *sepnorm)
{
	rbp_simplex s;

	/* Initial setup:
	 * Set first search direction as the vector between the objects
	 * calculate the first minkowski difference point and initiate the
	 * with that point (0-simplex)*/
	s.dir = SUB(b2->pos, b1->pos);
	s.B = rbp_support(b1, b2, s.dir);
	s.dir = NEG(s.B);
	s.n = 0;

	while(1) {
		/* Expand simplex dimension searching towards s.dir */
		s.A = rbp_support(b1, b2, s.dir);
		s.n++;

		/* If the search above returns a minkowski difference point that is
		 * not past the origin, we know we can't expand further. The shapes
		 * are disjoint and we don't have a collision. Set the last search
		 * direction as the separation normal and return a miss.
		 */
		if (DOT(NEG(s.A), s.dir) > 0) {
			*sepnorm = s.dir;
			return 0;
		}

		if (rbp_update_simplex(&s)) {
			/* This is only reached if rbp_u3simplex() returns 1,
			 * meaning that the origin is inside the tetrahedron s,
			 * which means we have a collision. */
			return 1;
		}
	}
}

/* Cleanup macros */
#undef NEG
#undef SUB
#undef DOT
#undef X
#undef X3