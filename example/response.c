#include <stdio.h>
#include <raylib.h>
#include <math.h>

#include <rbphys.h>

int main()
{
	InitWindow(640, 480, "rbphys");

	SetTargetFPS(120.0);
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
	planet.pos = (Vector3) {15.0f, 0.0f, 0.0f};
	planet.p = (Vector3) {0.0f, 0.0f, 1.0f};
	planet.dir = QuaternionIdentity();
	planet.L = (Vector3) {0.0f, -8.0f, 0.0f};
	planet.collider_type = SPHERE;
	rbp_sphere_collider planet_collider = {{0.0f, 0.0f, 0.0f}, 0.99f, 1.0f};
	planet.collider = &planet_collider;

	rbp_body sun;
	sun.Minv = 0.01f;
	sun.Ibinv = MatrixIdentity();
	sun.pos = Vector3Zero();
	sun.p = (Vector3) {0.0f, 0.0f, -0.001f};
	sun.dir = QuaternionIdentity();
	sun.L = Vector3Zero();
	sun.collider_type = SPHERE;
	rbp_sphere_collider sun_collider = {{0.0f, 0.0f, 0.0f}, 0.99f, 5.0f};
	sun.collider = &sun_collider;

	Camera3D camera = { 0 };
	camera.position = (Vector3) {0.0f, 40.0f, -1.0f};
	camera.target = Vector3Zero();
	camera.up = (Vector3) {0.0f, 1.0f, 0.0f};
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;

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

	rbp_contact contact;
	while(!WindowShouldClose()) {
		/* Update physics */
		now = GetTime();
		time_pool += now - time;
		time = now;

		while (time_pool >= dt) {
			Vector3 separation = Vector3Subtract(planet.pos, sun.pos);
			float r2 = Vector3DotProduct(separation, separation);
			Vector3 g = Vector3Normalize(separation);
			g = Vector3Scale(g, -1600.0f/r2);

			rbp_wspace_force(&planet, g, planet.pos, dt);
			rbp_wspace_force(&sun, Vector3Scale(g, -1.0f), sun.pos, dt);

			rbp_update(&planet, dt);
			rbp_update(&sun, dt);
			time_pool -= dt;

			if (rbp_collide(&planet, &sun, &contact)) {
				rbp_resolve_collision(&contact, dt);
			}
		}

		/* Update trajectory trace */
		trj_counter = (trj_counter + 1) % trj_max;
		trj[trj_counter] = planet.pos;
		trj_len = trj_len < trj_max ? trj_len+1 : trj_max;

		/* Update model and camera */
		planet_model.transform = QuaternionToMatrix(planet.dir);
		UpdateCamera(&camera, CAMERA_FREE);

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