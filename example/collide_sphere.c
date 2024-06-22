#include <stdio.h>
#include <raylib.h>
#include <math.h>

#include <rbphys.h>

int main()
{
	InitWindow(640, 480, "rbphys");

	SetTargetFPS(60.0);
	float dt = 1.0 / 60.0;

	Image checked = GenImageChecked(2, 2, 1, 1, GRAY, WHITE);
	Texture2D texture = LoadTextureFromImage(checked);
	UnloadImage(checked);

	Model planet_model = LoadModelFromMesh(GenMeshSphere(1.0f, 16, 16));
	Model sun_model = LoadModelFromMesh(GenMeshSphere(5.0f, 16, 16));
	planet_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
	sun_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;

	rbp_body planet;
	planet.m = 0.1f;
	planet.Ib = MatrixScale(0.1f, 0.1f, 0.1f);
	planet.pos = (Vector3) {10.0f, 0.0f, -9.0f};
	planet.p = (Vector3) {0.0f, 0.0f, 0.5f};
	planet.dir = QuaternionIdentity();
	planet.L = (Vector3) {0.0f, 0.0f, 0.0f};
	rbp_collider_sphere planet_collider = {
		.collider_type = SPHERE,
		.offset = Vector3Zero(),
		.e = 0.99f,
		.uf_s = 0.20f,
		.uf_d = 0.10f,
		.radius = 1.0f};
	planet.collider = &planet_collider;
	rbp_calculate_properties(&planet);

	rbp_body sun;
	sun.m = 10.0f;
	sun.Ib = MatrixScale(10.0f, 10.0f, 10.0f);
	sun.pos = (Vector3) {0.0f, 0.0f, -10.0f};
	sun.p = (Vector3) {0.0f, 0.0f, -0.5f};
	sun.dir = QuaternionIdentity();
	sun.L = Vector3Zero();
	rbp_collider_sphere sun_collider = {
		.collider_type = SPHERE,
		.offset = Vector3Zero(),
		.e = 0.99f,
		.uf_s = 0.20f, 
		.uf_d = 0.10f, 
		.radius = 5.0f};
	sun.collider = &sun_collider;
	rbp_calculate_properties(&sun);

	Vector3 offset = (Vector3) {0.0f, 0.0f, 20.0f};
	rbp_body planet2 = planet;
	planet2.pos = Vector3Add(planet2.pos, offset);
	rbp_collider_sphere planet2_collider = planet_collider;
	planet2.collider = &planet2_collider;

	rbp_body sun2 = sun;
	sun2.pos = Vector3Add(sun2.pos, offset);
	rbp_collider_sphere sun2_collider = sun_collider;
	sun2.collider = &sun2_collider;

	int trj1_max = 4096;
	unsigned int trj1_counter = 0;
	unsigned int trj1_len = 0;
	Vector3 trj1[trj1_max];
	trj1[0] = planet.pos;

	int trj2_max = 4096;
	unsigned int trj2_counter = 0;
	unsigned int trj2_len = 0;
	Vector3 trj2[trj2_max];
	trj2[0] = planet2.pos;

	rbp_contact contact1;
	rbp_contact contact2;

	Camera3D camera = { 0 };
	camera.position = (Vector3) {-20.0f, 50.0f, 0.0f};
	camera.target = Vector3Zero();
	camera.up = (Vector3) {0.0f, 1.0f, 0.0f};
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	double now;
	double time = GetTime();
	float time_pool = 0.0f;

	while(!WindowShouldClose()) {
		/* Update physics */
		now = GetTime();
		time_pool += now - time;
		time = now;

		while (time_pool >= dt) {
			Vector3 relpos1 = Vector3Subtract(planet.pos, sun.pos);
			Vector3 relpos2 = Vector3Subtract(planet2.pos, sun2.pos);
			float r21 = Vector3DotProduct(relpos1, relpos1);
			float r22 = Vector3DotProduct(relpos2, relpos2);
			Vector3 g1 = Vector3Normalize(relpos1);
			Vector3 g2 = Vector3Normalize(relpos2);
			g1 = Vector3Scale(g1, -160.0f/r21);
			g2 = Vector3Scale(g2, -160.0f/r22);

			rbp_wspace_force(&planet, g1, planet.pos, dt);
			rbp_wspace_force(&sun, Vector3Scale(g1, -1.0f), sun.pos, dt);

			rbp_wspace_force(&planet2, g2, planet2.pos, dt);
			rbp_wspace_force(&sun2, Vector3Scale(g2, -1.0f), sun2.pos, dt);

			rbp_update(&planet, dt);
			rbp_update(&planet2, dt);
			rbp_update(&sun, dt);
			rbp_update(&sun2, dt);
			time_pool -= dt;

			/* collide! */
			contact1.b1 = &planet;
			contact1.b2 = &sun;
			if (rbp_collide(&contact1)) {
				rbp_resolve_collision(&contact1, dt);
			}

			contact2.b1 = &planet2;
			contact2.b2 = &sun2;
			if (rbp_collide(&contact2)) {
				rbp_resolve_collision(&contact2, dt);
			}
		}

		/* Update trajectory trace */
		trj1_counter = (trj1_counter + 1) % trj1_max;
		trj1[trj1_counter] = planet.pos;
		trj1_len = trj1_len < trj1_max ? trj1_len+1 : trj1_max;

		trj2_counter = (trj2_counter + 1) % trj2_max;
		trj2[trj2_counter] = planet2.pos;
		trj2_len = trj2_len < trj2_max ? trj2_len+1 : trj2_max;

		/* Update model and camera */
		UpdateCamera(&camera, CAMERA_FREE);

		/* Render scene */
		BeginDrawing();
		ClearBackground(RAYWHITE);
			BeginMode3D(camera);

				for (int i=0; i<trj1_len-1; i++) {
					if (i != trj1_counter)
						DrawLine3D(trj1[i], trj1[i+1], RED);
				}

				for (int i=0; i<trj2_len-1; i++) {
					if (i != trj2_counter)
						DrawLine3D(trj2[i], trj2[i+1], GREEN);
				}

				planet_model.transform = QuaternionToMatrix(planet.dir);
				sun_model.transform = QuaternionToMatrix(sun.dir);
				DrawModel(sun_model, sun.pos, 1.0f, RED);
				DrawModel(planet_model, planet.pos, 1.0f, RED);

				planet_model.transform = QuaternionToMatrix(planet2.dir);
				sun_model.transform = QuaternionToMatrix(sun2.dir);
				DrawModel(sun_model, sun2.pos, 1.0f, GREEN);
				DrawModel(planet_model, planet2.pos, 1.0f, GREEN);

			EndMode3D();
			DrawFPS(1,1);
		EndDrawing();
	}

	CloseWindow();
	return 0;
}
