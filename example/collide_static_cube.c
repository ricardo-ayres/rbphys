#include <stdio.h>
#include <raylib.h>
#include <math.h>

#include <rbphys.h>

int main()
{
	InitWindow(800, 600, "rbphys");

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
	ball.m = 1.0f;
	ball.Ib = MatrixIdentity();
	ball.pos = (Vector3) {0.0f, 1.1f, -9.0f};
	ball.p = (Vector3) {0.0f, 0.0f, 16.0f};
	ball.dir = QuaternionIdentity();
	ball.L = (Vector3) {-16.0f, 0.0f, 0.0f};
	rbp_collider_sphere ball_collider ={
		.collider_type = SPHERE,
		.offset = (Vector3) {0.0f, 0.0f, 0.0f},
		.e = 0.90f,
		.uf_s = 0.6f,
		.uf_d = 0.3f,
		.radius = 1.0f};
	ball.collider = &ball_collider;
	rbp_calculate_properties(&ball);

	rbp_body ball1 = ball;
	rbp_body ball2 = ball;

	ball1.pos = (Vector3) {-6.0f, 1.1f, -9.0f};
	ball1.L = (Vector3) {-25.0f, 0.0f, 0.0f};

	ball2.pos = (Vector3) {+6.0f, 1.1f, -9.0f};
	ball2.L = (Vector3) {-9.0f, 0.0f, 0.0f};

	rbp_body *balls[] = {&ball, &ball1, &ball2};

	rbp_body slab;
	slab.m = 0.0f; /* static body */
	slab.pos = (Vector3) {0.0f, -1.0f, 0.0f};
	slab.p = Vector3Zero();
	slab.dir = QuaternionIdentity();
	slab.L = (Vector3) {0.0f, 0.0f, 0.0f};
	rbp_collider_cuboid slab_collider = {
		.collider_type = CUBOID,
		.offset = (Vector3) {0.0f,0.0f,0.0f},
		.e = 0.90f,
		.uf_s = 1.0f,
		.uf_d = 1.0f,
		.dir = QuaternionIdentity(),
		.xsize = 50.0f,
		.ysize = 2.0f,
		.zsize = 50.0f};
	slab.collider = &slab_collider;
	rbp_calculate_properties(&slab);

	Camera3D camera = { 0 };
	camera.position = (Vector3) {50.0f, 40.0f, 0.0f};
	camera.target = Vector3Zero();
	camera.up = (Vector3) {0.0f, 1.0f, 0.0f};
	camera.fovy = 45.0f;
	camera.projection = CAMERA_PERSPECTIVE;

	double now;
	double time = GetTime();
	float time_pool = 0.0f;
	
	Vector3 g = (Vector3) {0.0f, -10.0f, 0.0f};
	rbp_contact contact;
	while(!WindowShouldClose()) {
		/* Update physics */
		now = GetTime();
		time_pool += now - time;
		time = now;

		while (time_pool >= dt) {
			/* Update positions */

			time_pool -= dt;

			for (int i=0; i<3; i++) {
				rbp_body *b = balls[i];
				rbp_update(b, dt);
				contact.b1 = &slab;
				contact.b2 = b;
				if (rbp_collide(&contact)) {
					rbp_resolve_collision(&contact, dt);
				}

				/* Apply forces */
				rbp_wspace_force(b, g, b->pos, dt);
			}
		}

		/* Update model and camera */
		//ball_model.transform = QuaternionToMatrix(ball.dir);
		slab_model.transform = QuaternionToMatrix(slab.dir);
		UpdateCamera(&camera, CAMERA_FREE);

		/* Render scene */
		BeginDrawing();
		ClearBackground(RAYWHITE);
			BeginMode3D(camera);
				DrawModel(slab_model, slab.pos, 1.0f, RED);
				for (int i=0; i<3; i++) {
					rbp_body *b = balls[i];
					ball_model.transform = QuaternionToMatrix(b->dir);
					DrawModel(ball_model, b->pos, 1.0f, WHITE);
				}
			EndMode3D();
			DrawFPS(1,1);
		EndDrawing();
	}

	CloseWindow();
	return 0;
}
