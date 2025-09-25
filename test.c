#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

#define N 3488

typedef uint32_t fixed_point_t;
#define FIXED_POINT_FRACTIONAL_BITS 16

inline fixed_point_t double_to_fixed(double input) {
  return (fixed_point_t)(round(input * (1 << FIXED_POINT_FRACTIONAL_BITS)));
}

int main() {
  FILE* file = fopen( "teapot.tri", "r" );
  float a, b, c, d, e, f, g, h, i;
  printf("@000\n");
  for (int t = 0; t < N; t++)
  {
      int result = fscanf( file, "%f %f %f %f %f %f %f %f %f\n",
          &a, &b, &c, &d, &e, &f, &g, &h, &i );
      if (!result) exit(1);

      printf("%.8x\n", double_to_fixed((double)(a)));
      printf("%.8x\n", double_to_fixed((double)(b)));
      printf("%.8x\n", double_to_fixed((double)(c)));
      printf("%.8x\n", double_to_fixed((double)(d)));
      printf("%.8x\n", double_to_fixed((double)(e)));
      printf("%.8x\n", double_to_fixed((double)(f)));
      printf("%.8x\n", double_to_fixed((double)(g)));
      printf("%.8x\n", double_to_fixed((double)(h)));
      printf("%.8x\n", double_to_fixed((double)(i)));


  }
  fclose( file );
}

