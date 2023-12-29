#include <stdio.h>
#include <raylib.h>
#include <math.h>

#include <rbphys.h>

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
	planet.m = 0.1f;
	planet.Ib = MatrixScale(0.1f, 0.1f, 0.1f);
	planet.pos = (Vector3) {10.0f, 0.0f, 1.0f};
	planet.p = (Vector3) {0.0f, 0.0f, 0.5f};
	planet.dir = QuaternionIdentity();
	planet.L = (Vector3) {0.0f, 0.0f, 0.0f};
	rbp_collider_sphere planet_collider = {
		SPHERE,
		{0.0f, 0.0f, 0.0f},
		0.99f,
		0.20f,
		0.10f,
		1.00f,
		1.0f};
	planet.collider = &planet_collider;
	rbp_calculate_properties(&planet);

	rbp_body sun;
	sun.m = 10.0f;
	sun.Ib = MatrixScale(10.0f, 10.0f, 10.0f);
	sun.pos = Vector3Zero();
	sun.p = (Vector3) {0.0f, 0.0f, -0.5f};
	sun.dir = QuaternionIdentity();
	sun.L = Vector3Zero();
	rbp_collider_sphere sun_collider = {
		SPHERE,
		{0.0f, 0.0f, 0.0f}, 
		0.99f, 
		0.20f, 
		0.10f, 
		1.00f,
		5.0f};
	sun.collider = &sun_collider;
	rbp_calculate_properties(&sun);

	Camera3D camera = { 0 };
	camera.position = (Vector3) {-2.0f, 30.0f, -2.0f};
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
			Vector3 relpos = Vector3Subtract(planet.pos, sun.pos);
			float r2 = Vector3DotProduct(relpos, relpos);
			Vector3 g = Vector3Normalize(relpos);
			g = Vector3Scale(g, -160.0f/r2);
			rbp_wspace_force(&planet, g, planet.pos, dt);
			rbp_wspace_force(&sun, Vector3Scale(g, -1.0f), sun.pos, dt);

			rbp_update(&planet, dt);
			rbp_update(&sun, dt);
			time_pool -= dt;

			/* collide! */
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
		sun_model.transform = QuaternionToMatrix(sun.dir);
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
