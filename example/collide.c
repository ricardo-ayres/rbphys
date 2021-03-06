#include <stdio.h>
#include <raylib.h>
#include <math.h>

#include <rbphys.h>

Vector3
planet_collider(rbp_body *self, Vector3 d)
{
	Vector3 r = Vector3Scale(Vector3Normalize(d), 1.0f);
	return Vector3Add(self->pos, r);
}

Vector3
sun_collider(rbp_body *self, Vector3 d)
{
	Vector3 r = Vector3Scale(Vector3Normalize(d), 5.0f);
	return Vector3Add(self->pos, r);
}

int main()
{
	InitWindow(640, 480, "rbphys");

	SetTargetFPS(60.0);
	float dt = 1.0 / 60.0;

	Image checked = GenImageChecked(2, 2, 1, 1, RED, GREEN);
	Texture2D texture = LoadTextureFromImage(checked);
	UnloadImage(checked);

	Model planet_model = LoadModelFromMesh(GenMeshSphere(1.0f, 16, 16));
	Model sun_model = LoadModelFromMesh(GenMeshSphere(5.0f, 16, 16));
	planet_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
	sun_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;

	rbp_body planet;
	planet.Minv = 1.0f;
	planet.Ibinv = MatrixIdentity();
	planet.pos = (Vector3) {12.0f, 0.0f, 0.0f};
	planet.p = (Vector3) {0.0f, 2.0f, 9.7f};
	planet.dir = QuaternionIdentity();
	planet.L = (Vector3) {0.0f, -8.0f, 0.0f};
	planet.support = &planet_collider;

	rbp_body sun;
	sun.Minv = 0.1f;
	sun.Ibinv = MatrixIdentity();
	sun.pos = Vector3Zero();
	sun.p = Vector3Zero();
	sun.dir = QuaternionIdentity();
	sun.L = Vector3Zero();
	sun.support = &sun_collider;

	Camera3D camera = { 0 };
	camera.position = (Vector3) {-35.0f, 20.0f, -35.0f};
	camera.target = Vector3Zero();
	camera.up = (Vector3) {0.0f, 1.0f, 0.0f};
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;
	SetCameraMode(camera, CAMERA_FREE);

	double now;
	double time = GetTime();
	float time_pool = 0.0f;

	struct trajectory {
		Vector3 *p;
		struct trajectory *next;
	};

	int trj_max = 4096;
	unsigned int trj_counter = 0;
	unsigned int trj_len = 0;
	Vector3 trj[trj_max];
	trj[0] = planet.pos;

	while(!WindowShouldClose()) {
		/* Update physics */
		now = GetTime();
		time_pool += now - time;
		time = now;

		while (time_pool >= dt) {
			/* reset position and velocity if colliding */
			if (rbp_gjk(&planet, &sun)) {
				planet.pos = (Vector3) {12.0f, 0.0f, 0.0f};
				planet.p = (Vector3) {0.0f, 2.0f, 9.7f};
				trj_len = 0;
			}
			float r2 = Vector3DotProduct(planet.pos, planet.pos);
			Vector3 g = Vector3Normalize(planet.pos);
			Vector3 drag = Vector3Scale(planet.p, -0.01f);
			g = Vector3Scale(g, -1600.0f/r2);
			rbp_wspace_force(&planet, g, planet.pos, dt);
			rbp_wspace_force(&planet, drag, planet.pos, dt);

			planet.pos = rbp_displace(&planet, dt);
			planet.dir = rbp_rotate(&planet, dt);
			time_pool -= dt;
		}

		/* Update trajectory trace */
		trj_counter = (trj_counter + 1) % trj_max;
		trj[trj_counter] = planet.pos;
		trj_len = trj_len < trj_max ? trj_len+1 : trj_max;

		/* Update model and camera */
		planet_model.transform = QuaternionToMatrix(planet.dir);
		UpdateCamera(&camera);

		/* Render scene */
		BeginDrawing();
		ClearBackground(RAYWHITE);
			BeginMode3D(camera);
				DrawGrid(1000.0, 1.0f);
				for (int i=0; i<trj_len-1; i++) {
					if (i != trj_counter)
						DrawLine3D(trj[i], trj[i+1], RED);
				}
				DrawModel(sun_model, sun.pos, 1.0f, RED);
				DrawModel(planet_model, planet.pos, 1.0f, WHITE);
			EndMode3D();
			DrawFPS(1,1);
		EndDrawing();
	}

	CloseWindow();
	return 0;
}
