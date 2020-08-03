
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <stdio.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "../SingleBounceKernel/sbk.h"
#include "../cuda-raytracing-optimized/staircase_scene.h"

struct MortonKey
{
    signed int      oldSlot;
    unsigned int    hash[6];        // 192-bit Morton key
};

void computeTmax(saved_path* paths, uint32_t numpaths, bbox bounds);
void computeAABB(const saved_path* paths, uint32_t numpaths, bbox* aabb);
void computeMortonCodes(const saved_path* paths, uint32_t numpaths, MortonKey* keys);
void sortPaths(const saved_path* paths, uint32_t numpaths, const MortonKey* keys, saved_path* sorted);

int main(int argc, char** argv)
{
    int nx = 320;
    int ny = 400;
    int ns = 64;

    // extract bounce number from args
    int bounce = 0;
    if (argc > 1)
        bounce = strtol(argv[1], NULL, 10);
    std::string file = filename(bounce, ns);
    std::cerr << "loading bounce file " << file << std::endl;

    // load rays from bounce file
    uint32_t numpaths = nx * ny * ns;
    saved_path* paths = new saved_path[numpaths];
    if (!load(file, paths, numpaths)) {
        return -1;
    }

    // load scene's bvh and retrieve mesh bounds
    std::cerr << "loading scene" << std::endl;
    mesh* m = new mesh();
    int numPrimitivesPerLeaf = 0;
    if (!loadBVH("D:\\models\\obj\\staircase.bvh", *m, numPrimitivesPerLeaf)) {
        return -1;
    }
    bbox bounds = m->bounds;
    delete m;
    m = NULL;

    // compute tmax for all paths
    std::cerr << "computing tmax" << std::endl;
    computeTmax(paths, numpaths, bounds);

    // compute aabb for all paths
    std::cerr << "computing aabb" << std::endl;
    bbox* aabb = new bbox[numpaths];
    computeAABB(paths, numpaths, aabb);

    // compute Morton code for all rays
    std::cerr << "computing Morton codes" << std::endl;
    MortonKey* keys = new MortonKey[numpaths];
    computeMortonCodes(paths, numpaths, keys);

    // sort rays using their Morton codes
    std::cerr << "sorting paths" << std::endl;
    saved_path* sorted = new saved_path[numpaths];
    sortPaths(paths, numpaths, keys, sorted);

    // save rays to .sorted file
    std::string outfile = file + ".sorted";
    std::cerr << "saving file " << outfile << std::endl;
    save(outfile, sorted, numpaths);

    // cleanup
    delete[] paths;
    delete[] aabb;
    delete[] keys;
    delete[] sorted;

    return 0;
}

void computeTmax(saved_path* paths, uint32_t numpaths, bbox bounds) {
}

void computeAABB(const saved_path* paths, uint32_t numpaths, bbox* aabb) {
}

void computeMortonCodes(const saved_path* paths, uint32_t numpaths, MortonKey* keys) {
}

void sortPaths(const saved_path* paths, uint32_t numpaths, const MortonKey* keys, saved_path* sorted) {
}