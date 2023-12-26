#include "raylib.h"

int main(void)
{
    InitWindow(640, 480, "rbphys");

    // Define our custom camera to look into our 3d world
    Camera camera = { 0 };
    camera.position = (Vector3){ -150.0f, 150.0f, -150.0f };   // Camera position
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };         // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };              // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                    // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;                 // Camera projection type

    Image elevation_img = LoadImage("heightmap.png");       // Load heightmap image (RAM)
    Image texture_img = LoadImage("texture.png");
    Texture2D texture = LoadTextureFromImage(texture_img); // Convert image to texture (VRAM)

    Mesh heightmap_mesh = GenMeshHeightmap(elevation_img, (Vector3){ 16, 1, 16 });
    Model model = LoadModelFromMesh(heightmap_mesh);

    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture; // Set map diffuse texture
    Vector3 mapPosition = { 0.0f, 0.0f, 0.0f };           // Define model position

    UnloadImage(elevation_img);             // Unload heightmap image from RAM, already uploaded to VRAM
    UnloadImage(texture_img);             // Unload texture image from RAM, already uploaded to VRAM

    SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        UpdateCamera(&camera, CAMERA_FREE);
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(RAYWHITE);

            BeginMode3D(camera);

                DrawModel(model, mapPosition, 10.0f, WHITE);

            EndMode3D();

            //DrawTexture(texture, screenWidth - texture.width - 20, 20, WHITE);
            //DrawRectangleLines(screenWidth - texture.width - 20, 20, texture.width, texture.height, GREEN);

            DrawFPS(10, 10);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    UnloadTexture(texture);     // Unload texture
    UnloadModel(model);         // Unload model

    CloseWindow();              // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}