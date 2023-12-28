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

	Model ball_model = LoadModelFromMesh(GenMeshSphere(1.0f, 16, 16));
	Model slab_model = LoadModelFromMesh(GenMeshCube(50.0f, 2.0f, 50.0f));
	ball_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
	slab_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;

	rbp_body ball;
	ball.Minv = 1.0f;
	ball.Ibinv = MatrixIdentity();
	ball.pos = (Vector3) {5.0f, 5.0f, 0.0f};
	ball.p = (Vector3) {-50.0f, -50.0f, 0.0f};
	ball.dir = QuaternionIdentity();
	ball.L = (Vector3) {0.0f, 0.0f, 0.0f};
	rbp_collider_sphere ball_collider ={
		SPHERE,
		{0.0f, 0.0f, 0.0f},
		0.99f,
		0.40f,
		0.30f,
		1.0f};
	ball.collider = &ball_collider;

	rbp_body tabletop;
	tabletop.Minv = 0.0f;
	tabletop.Ibinv = MatrixScale(0.00f, 0.00f, 0.00f);
	tabletop.Ibinv.m15 = 1.0f;
	tabletop.pos = (Vector3) {0.0f, 10.0f, 0.0f};
	tabletop.p = Vector3Zero();
	tabletop.dir = QuaternionIdentity();
	tabletop.L = (Vector3) {0.0f, 0.0f, 0.0f};
	rbp_collider_cuboid tabletop_collider = {
		CUBOID,
		{0.0f,0.0f,0.0f},
		0.90f,
		0.10f,
		0.08f,
		QuaternionIdentity(),
		50.0f,
		2.0f,
		50.0f};
	tabletop.collider = &tabletop_collider;

	rbp_body floor;
	floor.Minv = 0.0f;
	floor.Ibinv = MatrixScale(0.00f, 0.00f, 0.00f);
	floor.Ibinv.m15 = 1.0f;
	floor.pos = (Vector3) {0.0f, -1.0f, 0.0f};
	floor.p = Vector3Zero();
	floor.dir = QuaternionIdentity();
	floor.L = (Vector3) {0.0f, 0.0f, 0.0f};
	rbp_collider_cuboid floor_collider = {
		CUBOID,
		{0.0f,0.0f,0.0f},
		0.90f,
		0.10f,
		0.08f,
		QuaternionIdentity(),
		50.0f,
		2.0f,
		50.0f};
	floor.collider = &floor_collider;

	Camera3D camera = { 0 };
	camera.position = (Vector3) {0.0f, 5.0f, -40.0f};
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
	trj[0] = ball.pos;

	Vector3 g = (Vector3) {0.0f, -10.0f/ball.Minv, 0.0f};
	rbp_contact contact;
	while(!WindowShouldClose()) {
		/* Update physics */
		now = GetTime();
		time_pool += now - time;
		time = now;

		while (time_pool >= dt) {
			rbp_wspace_force(&ball, g, ball.pos, dt);
			rbp_update(&ball, dt);
			time_pool -= dt;

			/* Check collisions, reset ball if hit */
			if (rbp_collide(&ball, &floor, &contact)) {
				rbp_resolve_collision(&contact, dt);
			}
			if (rbp_collide(&ball, &tabletop, &contact)) {
				rbp_resolve_collision(&contact, dt);
			}
		}

		/* Update trajectory trace */
		trj_counter = (trj_counter + 1) % trj_max;
		trj[trj_counter] = ball.pos;
		trj_len = trj_len < trj_max ? trj_len+1 : trj_max;

		/* Update model and camera */
		ball_model.transform = QuaternionToMatrix(ball.dir);
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
				DrawModel(slab_model, tabletop.pos, 1.0f, GREEN);
				DrawModel(slab_model, floor.pos, 1.0f, RED);
				DrawModel(ball_model, ball.pos, 1.0f, WHITE);
			EndMode3D();
			DrawFPS(1,1);
		EndDrawing();
	}

	CloseWindow();
	return 0;
}
