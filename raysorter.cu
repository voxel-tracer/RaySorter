
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <stdio.h>
#include <random>

#define DEBUG_SORT

#define SAVE_BITSTACK

#define STB_IMAGE_IMPLEMENTATION
#include "../cuda-raytracing-optimized/stb_image.h"

#include "../SingleBounceKernel/sbk.h"
#include "../cuda-raytracing-optimized/staircase_scene.h"

struct MortonKey
{
    int         oldSlot;
    uint32_t    hash[6];        // 192-bit Morton key
};

struct light_path {
    vec3 origin;
    vec3 direction;
    float tmax;

    light_path() {}
    light_path(const saved_path& p) : origin(p.origin), direction(p.rayDir), tmax(FLT_MAX) {}
};

void computeTmax(light_path* paths, uint32_t numpaths, bbox bounds);
bbox computeAABB(const light_path* paths, uint32_t numpaths);
void computeMortonCodes(const light_path* paths, uint32_t numpaths, const bbox aabb, MortonKey* keys);
void sortPaths(const saved_path* paths, uint32_t numpaths, MortonKey* keys, saved_path* sorted);

bool tmax_bbox(const bbox& bounds, light_path& p);
void collectBits(unsigned int* hash, int index, unsigned int value);
int compareMortonKey(const void* A, const void* B);
int compareSavedPath(const void* A, const void* B);

int main(int argc, char** argv)
{
    int nx = 320;
    int ny = 400;
    int ns = 64;

    // extract bounce number from args
    int bounce = 0;
    if (argc > 1)
        bounce = strtol(argv[1], NULL, 10);
    std::string file = filename(bounce, ns, false);
    std::cerr << "loading bounce file " << file << std::endl;

    // load rays from bounce file
    uint32_t numpaths = nx * ny * ns;
    saved_path* paths = new saved_path[numpaths];
    light_path* light = new light_path[numpaths];
    if (!load(file, paths, numpaths)) {
        return -1;
    }
    //// replace bitstack with a random number
    //std::random_device rd;
    //std::mt19937 gen(rd());
    //for (auto i = 0; i < numpaths; i++) {
    //    paths[i].bitstack = gen();
    //}

    for (auto i = 0; i < numpaths; i++) {
        light[i] = light_path(paths[i]);
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
    computeTmax(light, numpaths, bounds);

    // compute aabb for all paths
    std::cerr << "computing aabb" << std::endl;
    bbox aabb = computeAABB(light, numpaths);

    // compute Morton code for all rays
    std::cerr << "computing Morton codes" << std::endl;
    MortonKey* keys = new MortonKey[numpaths];
    computeMortonCodes(light, numpaths, aabb, keys);

    // sort rays using their Morton codes
    std::cerr << "sorting paths" << std::endl;
    saved_path* sorted = new saved_path[numpaths];
    sortPaths(paths, numpaths, keys, sorted);

    // save rays to .sorted file
    std::string outfile = filename(bounce, ns, true);
    std::cerr << "saving file " << outfile << std::endl;
    save(outfile, sorted, numpaths);

    //qsort(paths, numpaths, sizeof(saved_path), compareSavedPath);

    //// save rays to .sorted file
    //std::string outfile = filename(bounce, ns, true);
    //std::cerr << "saving file " << outfile << std::endl;
    //save(outfile, paths, numpaths);

    // cleanup
    delete[] paths;
    delete[] light;
    delete[] keys;
    delete[] sorted;

    return 0;
}

void computeTmax(light_path* paths, uint32_t numpaths, bbox bounds) {
    for (auto i = 0; i < numpaths; i++) {
        //tmax_bbox(bounds, paths[i]);
        paths[i].tmax = 1.0f;
    }
}

bbox computeAABB(const light_path* paths, uint32_t numpaths) {
    bbox aabb;
    aabb.min = vec3(FLT_MAX, FLT_MAX, FLT_MAX);
    aabb.max = vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

#ifdef DEBUG_SORT
    bool first = true;
#endif
    for (auto i = 0; i < numpaths; i++) {
        const light_path& p = paths[i];
        aabb.min = min(aabb.min, p.origin);
        aabb.max = max(aabb.max, p.origin);

        vec3 far = p.origin + p.tmax * p.direction;
        aabb.min = min(aabb.min, far);
        aabb.max = max(aabb.max, far);
#ifdef DEBUG_SORT
        if (aabb.min[0] == -FLT_MAX && first) {
            std::cerr << "aabb.min = " << aabb.min << std::endl;
            std::cerr << "i = " << i << std::endl;
            std::cerr << "p.origin = " << p.origin << std::endl;
            std::cerr << "p.direction = " << p.direction << std::endl;
            first = false;
        }
#endif
    }
    return aabb;
}

void computeMortonCodes(const light_path* paths, uint32_t numpaths, const bbox aabb, MortonKey* keys) {
    for (auto i = 0; i < numpaths; i++) {
        const light_path& p = paths[i];
        MortonKey& key = keys[i];

        // normalize origin and direction
        vec3 o = (p.origin - aabb.min) / (aabb.max - aabb.min);
        vec3 d = (p.direction - aabb.min) / (aabb.max - aabb.min);

        // generate hash
        key.oldSlot = i;
        for (auto j = 0; j < 6; j++)
            key.hash[j] = 0;
        collectBits(key.hash, 0, (uint32_t)(o[0] * 256.0f * 65536.0f));
        collectBits(key.hash, 1, (uint32_t)(o[1] * 256.0f * 65536.0f));
        collectBits(key.hash, 2, (uint32_t)(o[2] * 256.0f * 65536.0f));
        collectBits(key.hash, 3, (uint32_t)(d[0] * 32.0f * 65536.0f));
        collectBits(key.hash, 4, (uint32_t)(d[1] * 32.0f * 65536.0f));
        collectBits(key.hash, 5, (uint32_t)(d[2] * 32.0f * 65536.0f));
#ifdef DEBUG_SORT
        if (i == 500) {
            std::cerr << "key.oldSlot = " << key.oldSlot << std::endl;
            std::cerr << "key.hash = ";
            for (size_t j = 0; j < 6; j++)
                std::cerr << key.hash[j] << " ";
            std::cerr << std::endl;
            std::cerr << "aabb.min = " << aabb.min << std::endl;
            std::cerr << "aabb.max = " << aabb.max << std::endl;
        }
#endif // DEBUG_SORT
    }
}

void sortPaths(const saved_path* paths, uint32_t numpaths, MortonKey* keys, saved_path* sorted) {
    // sort Morton keys
    qsort(keys, numpaths, sizeof(MortonKey), compareMortonKey);

    // reorder paths using their sort order
    for (auto i = 0; i < numpaths; i++) {
        sorted[i] = paths[keys[i].oldSlot];
    }
}

bool tmax_bbox(const bbox& bounds, light_path& p) {
    float t_min = 0.001f;
    p.tmax = FLT_MAX;
    for (int a = 0; a < 3; a++) {
        float invD = 1.0f / p.direction[a];
        float t0 = (bounds.min[a] - p.origin[a]) * invD;
        float t1 = (bounds.max[a] - p.origin[a]) * invD;
        if (invD < 0.0f) {
            float tmp = t0; t0 = t1; t1 = tmp;
        }
        t_min = t0 > t_min ? t0 : t_min;
        p.tmax = t1 < p.tmax ? t1 : p.tmax;
        if (p.tmax < t_min)
            return false;
    }

    return true;
}

void collectBits(unsigned int* hash, int index, unsigned int value) {
    for (int i = 0; i < 32; i++)
        hash[(index + i * 6) >> 5] |= ((value >> i) & 1) << ((index + i * 6) & 31);
}

int compareMortonKey(const void* A, const void* B) {
    const MortonKey& a = *((const MortonKey*)A);
    const MortonKey& b = *((const MortonKey*)B);
    if (a.hash[5] != b.hash[5]) return (a.hash[5] < b.hash[5]) ? -1 : 1;
    if (a.hash[4] != b.hash[4]) return (a.hash[4] < b.hash[4]) ? -1 : 1;
    if (a.hash[3] != b.hash[3]) return (a.hash[3] < b.hash[3]) ? -1 : 1;
    if (a.hash[2] != b.hash[2]) return (a.hash[2] < b.hash[2]) ? -1 : 1;
    if (a.hash[1] != b.hash[1]) return (a.hash[1] < b.hash[1]) ? -1 : 1;
    if (a.hash[0] != b.hash[0]) return (a.hash[0] < b.hash[0]) ? -1 : 1;
    return 0;
}

int compareSavedPath(const void* A, const void* B) {
    const saved_path& a = *((const saved_path*)A);
    const saved_path& b = *((const saved_path*)B);
    if (a.bitstack == b.bitstack) return 0;
    return a.bitstack < b.bitstack ? -1 : 1;
}
