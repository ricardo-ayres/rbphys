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