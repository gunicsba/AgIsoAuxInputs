// Host program for tools/triton_pool.py: builds the NeptuneGPS_Triton plough
// controller's VT object pool with its own BuildObjectPool() (against its
// native Arduino stubs) and writes it to the file given as argument.
#include "isobus/VTObjectPool.hpp"

#include <cstdio>

int main(int argc, char **argv) {
  if (argc != 2)
    return 2;
  triton::BuildObjectPool();
  FILE *out = std::fopen(argv[1], "wb");
  if (!out)
    return 1;
  const bool ok = std::fwrite(triton::VT3PoolData, 1, triton::VT3PoolSize, out) == triton::VT3PoolSize;
  return (std::fclose(out) == 0 && ok) ? 0 : 1;
}
