#include <raylib.h>
#include <stdint.h>
#include <stdio.h>


#define screenWidth 640
#define screenHeight 480

static Color draw[screenWidth][screenHeight];

void initRaylib() {
  InitWindow(screenWidth, screenHeight, "raylib windows");
  SetTargetFPS(60);
}

// Draw four pixels (bluespec see the screen as a size 320*240)
void drawRaylibPoint(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
  draw[2*x+1][2*y+1] = (Color){r,g,b,255};
  draw[2*x][2*y+1] = (Color){r,g,b,255};
  draw[2*x+1][2*y] = (Color){r,g,b,255};
  draw[2*x][2*y] = (Color){r,g,b,255};
}

bool mustExitRaylib() {
  return WindowShouldClose();
}

void drawRaylib() {
  BeginDrawing();

  ClearBackground(RAYWHITE);

  for (int i=0; i<screenWidth; i++) {
    for (int j=0; j<screenHeight; j++) {
      DrawPixel(i,j,draw[i][j]);
    }
  }

  EndDrawing();
}

void exitRaylib() {
  CloseWindow();
}
