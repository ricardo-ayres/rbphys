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
	Model slab_model = LoadModelFromMesh(GenMeshCube(50.0f, 2.0f, 50.0f));
	cube_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
	slab_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;

	rbp_body cube;
	cube.m = 1.0f;
	cube.Ib = MatrixIdentity();
	cube.pos = (Vector3) {0.0f, 2.0f, 0.0f};
	cube.p = (Vector3) {0.0f, 0.0f, 0.0f};
	cube.dir = QuaternionFromAxisAngle((Vector3) {1.0f, 1.0f, 1.0f}, PI*0.25f);
	cube.L = (Vector3) {1.0f, 1.0f, -1.0f};
	rbp_collider_cuboid cube_collider = {
		.collider_type = CUBOID,
		.offset = Vector3Zero(),
		.e = 0.80f,
		.uf_s = 0.6f,
		.uf_d = 0.4f,
		.dir = QuaternionIdentity(),
		.xsize = 1.0f,
		.ysize = 1.0f,
		.zsize = 1.0f};
	cube.collider = &cube_collider;

	rbp_body slab;
	slab.m = 0.0f;
	slab.Ib = MatrixIdentity();
	slab.pos = (Vector3) {0.0f, -2.0f, 0.0f};
	slab.p = (Vector3) {0.0f, 0.0f, 0.0f};
	slab.dir = QuaternionIdentity();
	slab.L = Vector3Zero();
	rbp_collider_cuboid slab_collider = {
		.collider_type = CUBOID,
		.offset = Vector3Zero(),
		.e = 1.0f,
		.uf_s = 1.0f,
		.uf_d = 1.0f,
		.dir = QuaternionIdentity(),
		.xsize = 50.0f,
		.ysize = 1.0f,
		.zsize = 50.0f};
	slab.collider = &slab_collider;

	rbp_calculate_properties(&cube);
	rbp_calculate_properties(&slab);

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

		/* Update physics */
		now = GetTime();
		time_pool += now - time;
		time = now;

		while (time_pool >= dt) {
			rbp_update(&cube, dt);
			time_pool -= dt;
		}

		/* Apply gravity */
		rbp_wspace_force(&cube, (Vector3) {0.0f, -10.0f, 0.0f}, cube.pos, dt);

		/* Update model and camera */
		cube_model.transform = QuaternionToMatrix(cube.dir);
		UpdateCamera(&camera, CAMERA_FREE);

		/* Render scene */
		BeginDrawing();
		ClearBackground(RAYWHITE);
			BeginMode3D(camera);
				DrawModel(slab_model, slab.pos, 1.0f, RED);
				DrawModel(cube_model, cube.pos, 1.0f, WHITE);
			EndMode3D();
			DrawFPS(1,1);
		EndDrawing();
	}

	CloseWindow();
	return 0;
}
