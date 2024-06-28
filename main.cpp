#include <Novice.h>
#include<imgui.h>
#include"MyMath.h"
#include "Vector3.h"
#include "Matrix4x4.h"

static const int kWindowWidth = 1280; 
static const int kWindowHeight = 720;

const char kWindowTitle[] = "LE2C_19_トウ_カグン";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

    // ライブラリの初期化
    Novice::Initialize(kWindowTitle, 1280, 720);

    const float moveSpeed = 0.01f;

    Vector3 cameraTranslate = {0.0f, 1.9f, -6.49f};
    Vector3 cameraRotate = {0.26f, 0.0f, 0.0f};
    Vector3 viewTranslate = {0.0f, 0.0f, 0.0f};
    Vector3 cameraScale = {1.0f, 1.0f, 1.0f};

    Sphere spheres[2] = {
        {{0.0f, 0.0f, 0.0f}, 0.6f},
        {{1.0f, 1.0f, 1.0f}, 0.4f}
    };

    bool isDraggingMiddle = false;
    int lastMousePosX = 0, lastMousePosY = 0;
    // キー入力結果を受け取る箱
    char keys[256] = {0};
    char preKeys[256] = {0};

    // ウィンドウの×ボタンが押されるまでループ
    while (Novice::ProcessMessage() == 0) {
        // フレームの开始
        Novice::BeginFrame();

        // キー入力を受け取る
        memcpy(preKeys, keys, 256);
        Novice::GetHitKeyStateAll(keys);

        ///
        /// ↓更新処理ここから
        ///
        ///カメラ移動
        if (keys[DIK_W]) {
            cameraTranslate.z += moveSpeed;
        }
        if (keys[DIK_S]) {
            cameraTranslate.z -= moveSpeed;
        }
        if (keys[DIK_A]) {
            cameraTranslate.x -= moveSpeed;
        }
        if (keys[DIK_D]) {
            cameraTranslate.x += moveSpeed;
        }

        int mouseX, mouseY;
        Novice::GetMousePosition(&mouseX, &mouseY);
        if (Novice::IsPressMouse(2)) { 
            if (!isDraggingMiddle) {
                isDraggingMiddle = true;
                lastMousePosX = mouseX;
                lastMousePosY = mouseY;
            } else {
                int deltaX = mouseX - lastMousePosX;
                int deltaY = mouseY - lastMousePosY;

                cameraRotate.y += deltaX * 0.001f;
                cameraRotate.x += deltaY * 0.001f;

                lastMousePosX = mouseX;
                lastMousePosY = mouseY;
            }
        } else {
            isDraggingMiddle = false;
        }

        int wheel = Novice::GetWheel();
        Vector3 forward = {
            -sin(cameraRotate.y) * cos(cameraRotate.x),
            sin(cameraRotate.x),
            -cos(cameraRotate.y) * cos(cameraRotate.x)
        };

        cameraTranslate.x -= forward.x * wheel * 0.001f;
        cameraTranslate.y -= forward.y * wheel * 0.001f;
        cameraTranslate.z -= forward.z * wheel * 0.001f;
        
         Matrix4x4 viewMatrix = MakeAffineMatrix({1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, viewTranslate);
        Matrix4x4 cameraMatrix = MakeAffineMatrix(cameraScale, cameraRotate, cameraTranslate);
        Matrix4x4 combinedViewMatrix = Multiply(viewMatrix, Inverse(cameraMatrix));
        Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
        Matrix4x4 viewProjectionMatrix = Multiply(combinedViewMatrix, projectionMatrix);
        Matrix4x4 viewportMatrix = MakeViewportMatrix(0, 0, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);
        ///
        /// ↑更新処理ここまで
        ///

        ///
        /// ↓描画処理ここから
        ///

        bool collision = IsCollision(spheres[0], spheres[1]);
        uint32_t color = collision ? 0xFF0000FF : 0xFFFFFFFF;

        DrawGrid(viewProjectionMatrix, viewportMatrix);
        DrawSphere(spheres[0], viewProjectionMatrix, viewportMatrix, 0xFFFFFFFF);
        DrawSphere(spheres[1], viewProjectionMatrix, viewportMatrix, color);
        ImGui::Begin("Window");
        ImGui::DragFloat3("Sphere[0].Center", &spheres[0].center.x, 0.01f);
        ImGui::DragFloat("Sphere[0].Radius", &spheres[0].radius, 0.01f);
        ImGui::DragFloat3("Sphere[1].Center", &spheres[1].center.x, 0.01f);
        ImGui::DragFloat("Sphere[1].Radius", &spheres[1].radius, 0.01f);
        ImGui::End();
        ///
        /// ↑描画処理ここまで
        ///

        // フレームの終了
        Novice::EndFrame();

        // ESCキーが押されたらループを抜ける
        if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
            break;
        }
    }

    // ライブラリの終了
    Novice::Finalize();
    return 0;
}
