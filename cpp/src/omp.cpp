/*
It is more probable that earlier indices execute earlier only when the schedule keeps neighbors together in the same chunk. 
Otherwise, across threads, there is no bias — the probability is roughly equal for either order.
*/

#include <omp.h>

#include <cstdio>

int main(int argc, char **argv)
{
  // By default, OpenMP will try to use all logical cores (hardware threads) detected by the system.
#pragma omp parallel
  {
    printf("omp\n");
  }

  int total = 0; // declaration (shared by all objects)

// The content in parentheses is the name of the critical region. This name defines which critical regions share the same lock.
#pragma omp critical(update_data)
  {
    ++total; // valid: accessing static member from member function
  }


  int cores = omp_get_num_procs(); // number of logical processors
  omp_set_num_threads(cores);
#pragma omp parallel num_threads(omp_get_num_procs())
  {
    // ...
  }

  // #pragma omp single : Only one of those threads executes the following block
  // #pragma omp single nowait : Same, but others don’t wait afterward
  return 0;
}