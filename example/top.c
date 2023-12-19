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

	Model cube_model = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
	cube_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;

	rbp_body cube;
	cube.Minv = 1.0f;
	cube.Ibinv = MatrixIdentity();
	cube.pos = (Vector3) {0.0f, 0.0f, 0.0f};
	cube.p = (Vector3) {0.0f, 0.0f, 0.0f};
	cube.dir = QuaternionFromAxisAngle((Vector3) {0.0f, 0.0f, 1.0f}, PI*0.1);
	cube.L = (Vector3) {0.0f, 0.0f, 0.0f};

	Camera3D camera = { 0 };
	camera.position = Vector3Add(cube.pos, (Vector3) {-1.0f, 2.0f, -5.0f});
	camera.target = cube.pos;
	camera.up = (Vector3) {0.0f, 1.0f, 0.0f};
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	double now;
	double time = GetTime();
	float time_pool = 0.0f;

	while(!WindowShouldClose()) {
		/* Input */
		if (IsKeyDown(KEY_L)) {
			/* Increase angular momentum */
			rbp_bspace_force(
			    &cube,
			    (Vector3) {1.0f, 0.0f, 0.0f},
			    (Vector3) {0.0f, 0.0f, 1.0f},
			    dt);
			rbp_bspace_force(
			    &cube,
			    (Vector3) {-1.0f, 0.0f, 0.0f},
			    (Vector3) {0.0f, 0.0f, -1.0f},
			    dt);
		}
		if (IsKeyDown(KEY_K)) {
			/* Simulate top contact point */
			rbp_wspace_force(
			    &cube,
			    (Vector3) {0.0f, 10.0f, 0.0f},
			    rbp_wtobspace(&cube, (Vector3) {0.0f, -1.0f, 0.0f}),
			    dt);

			/* Simulate gravity */
			rbp_wspace_force(&cube,
			    (Vector3) {0.0f, -10.0f, 0.0f},
			    cube.pos,
			    dt);
		}

		/* Update physics */
		now = GetTime();
		time_pool += now - time;
		time = now;

		while (time_pool >= dt) {
			cube.pos = rbp_displace(&cube, dt);
			cube.dir = rbp_rotate(&cube, dt);
			time_pool -= dt;
		}

		/* Update model and camera */
		cube_model.transform = QuaternionToMatrix(cube.dir);
		UpdateCamera(&camera, CAMERA_FREE);

		/* Render scene */
		BeginDrawing();
		ClearBackground(RAYWHITE);
			BeginMode3D(camera);
				DrawGrid(1000.0, 1.0f);
				DrawModel(cube_model, cube.pos, 1.0f, WHITE);
			EndMode3D();
			DrawFPS(1,1);
		EndDrawing();
	}

	CloseWindow();
	return 0;
}
