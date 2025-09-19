#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <SDL2/SDL.h>


#define screenWidth 1280
#define screenHeight 960

typedef struct Color {
  unsigned char r;
  unsigned char g;
  unsigned char b;
} Color;

static Color draw[screenWidth][screenHeight];

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;

void init() {
  window = SDL_CreateWindow(
    "ulx3s simulation",
    SDL_WINDOWPOS_CENTERED,
    SDL_WINDOWPOS_CENTERED,
    screenWidth,
    screenHeight,
    0
  );

  if (!window) exit(1);

  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

  if (!renderer) exit(1);
}

// Draw four pixels (bluespec see the screen as a size 320*240)
void setPoint(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
  for (int i=0; i < 4; i++) {
    for (int j=0; j < 4; j++) {
      draw[4*x+i][4*y+j] = (Color){r,g,b};
    }
  }
}

void drawScreen() {
  for (int i=0; i < screenWidth; i++) {
    for (int j=0; j < screenHeight; j++) {
      SDL_SetRenderDrawColor(renderer, draw[i][j].r, draw[i][j].g, draw[i][j].b, 255);
      SDL_RenderDrawPoint(renderer, i, j);
    }
  }

  SDL_RenderPresent(renderer);
}
