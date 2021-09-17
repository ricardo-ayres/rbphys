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

	Model sphere_model = LoadModelFromMesh(GenMeshSphere(1.0f, 16, 16));
	Model sun_model = LoadModelFromMesh(GenMeshSphere(5.0f, 16, 16));
	sphere_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
	sun_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;

	rbp_body sphere;
	sphere.Minv = 1.0f;
	sphere.Ibinv = MatrixIdentity();
	sphere.pos = (Vector3) {12.0f, 0.0f, 0.0f};
	sphere.p = (Vector3) {0.0f, 2.0f, 13.2f};
	sphere.dir = QuaternionIdentity();
	sphere.L = (Vector3) {0.0f, -8.0f, 0.0f};

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
	trj[0] = sphere.pos;

	while(!WindowShouldClose()) {
		/* Update physics */
		now = GetTime();
		time_pool += now - time;
		time = now;

		while (time_pool >= dt) {
			float r2 = Vector3Length(sphere.pos);
			r2 = r2*r2;
			Vector3 g = Vector3Normalize(sphere.pos);
			g = Vector3Scale(g, -1600.0f/r2);
			rbp_wspace_force(&sphere, g, sphere.pos, dt);

			sphere.pos = rbp_displace(&sphere, dt);
			sphere.dir = rbp_rotate(&sphere, dt);
			time_pool -= dt;
		}

		/* Update trajectory trace */
		trj_counter = (trj_counter + 1) % trj_max;
		trj[trj_counter] = sphere.pos;
		trj_len = trj_len < trj_max ? trj_len+1 : trj_max;

		/* Update model and camera */
		sphere_model.transform = QuaternionToMatrix(sphere.dir);
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
				DrawModel(sun_model, Vector3Zero(), 1.0f, RED);
				DrawModel(sphere_model, sphere.pos, 1.0f, WHITE);
			EndMode3D();
			DrawFPS(1,1);
		EndDrawing();
	}

	CloseWindow();
	return 0;
}
