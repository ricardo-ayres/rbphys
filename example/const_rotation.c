/* flatspin.c */
#include <stdio.h>
#include <raylib.h>

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
	cube.rot = MatrixIdentity();
	cube.L = (Vector3) {2.0f, 1.0f, 0.0f};

	Camera3D camera = { 0 };
	camera.position = Vector3Add(cube.pos, (Vector3) {-1.0f, 2.0f, -5.0f});
	camera.target = cube.pos;
	camera.up = (Vector3) {0.0f, 1.0f, 0.0f};
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;
	SetCameraMode(camera, CAMERA_FREE);

	double now;
	double time = GetTime();
	float time_pool = 0.0f;

	while(!WindowShouldClose()) {

		/* Update physics */
		now = GetTime();
		time_pool += now - time;
		time = now;

		while (time_pool >= dt) {
			cube.rot = drotdt(&cube, dt);
			time_pool -= dt;
		}

		/* Update model and camera */
		cube_model.transform = cube.rot;
		UpdateCamera(&camera);

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
