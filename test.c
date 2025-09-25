#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <SDL2/SDL.h>


#define screenWidth 1280
#define screenHeight 960

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static SDL_Event *event = NULL;

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



#define N 3488

typedef uint32_t fixed_point_t;
#define FIXED_POINT_FRACTIONAL_BITS 16

inline fixed_point_t double_to_fixed(double input) {
  return (fixed_point_t)(round(input * (1 << FIXED_POINT_FRACTIONAL_BITS)));
}

int main() {
  init();
  SDL_SetRenderDrawColor(renderer, 100, 100, 0, 255);

  FILE* file = fopen( "teapot.tri", "r" );
  float a, b, c, d, e, f, g, h, i;


  printf("@000\n");
  for (int t = 0; t < N; t++)
  {
      int result = fscanf( file, "%f %f %f\n%f %f %f\n%f %f %f\n\n",
          &a, &b, &c, &d, &e, &f, &g, &h, &i );

      printf("%.8x\n", double_to_fixed((double)(a)));
      printf("%.8x\n", double_to_fixed((double)(b)));
      printf("%.8x\n", double_to_fixed((double)(c)));
      printf("%.8x\n", double_to_fixed((double)(d)));
      printf("%.8x\n", double_to_fixed((double)(e)));
      printf("%.8x\n", double_to_fixed((double)(f)));
      printf("%.8x\n", double_to_fixed((double)(g)));
      printf("%.8x\n", double_to_fixed((double)(h)));
      printf("%.8x\n", double_to_fixed((double)(i)));


      int i = (int)(round((a) * 100 + 500));
      int j = (int)(round((b) * 100 + 500));
      SDL_RenderDrawPoint(renderer, i, j);

      i = (int)(round((d) * 100 + 500));
      j = (int)(round((e) * 100 + 500));
      SDL_RenderDrawPoint(renderer, i, j);
  }
  fclose( file );

  SDL_RenderPresent(renderer);

  while (SDL_PollEvent(event)) {
    switch (event->type) {
      case SDL_WINDOWEVENT:
        if (event->window.event == SDL_WINDOWEVENT_CLOSE) {
          printf("close window\n");
          exit(1);
        }
      default: continue;
    }
  }
}

