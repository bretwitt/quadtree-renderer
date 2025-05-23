#ifndef QTPLANET_PERLIN_H
#define QTPLANET_PERLIN_H
//----------------------------------------------------------
// A simple Perlin noise implementation in 2D.
// (This is a basic version; for more robust noise consider using an external library.)
//----------------------------------------------------------
namespace Perlin {
    // Permutation table. The same list is repeated twice.
    static int permutation[256] = {
        151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,
        140,36,103,30,69,142,8,99,37,240,21,10,23,190,6,148,
        247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,
        57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,
        74,165,71,134,139,48,27,166,77,146,158,231,83,111,229,122,
        60,211,133,230,220,105,92,41,55,46,245,40,244,102,143,54,
        65,25,63,161,1,216,80,73,209,76,132,187,208,89,18,169,
        200,196,135,130,116,188,159,86,164,100,109,198,173,186,3,64,
        52,217,226,250,124,123,5,202,38,147,118,126,255,82,85,212,
        207,206,59,227,47,16,58,17,182,189,28,42,223,183,170,213,
        119,248,152,2,44,154,163,70,221,153,101,155,167,43,172,9,
        129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,
        218,246,97,228,251,34,242,193,238,210,144,12,191,179,162,241,
        81,51,145,235,249,14,239,107,49,192,214,31,181,199,106,157,
        184,84,204,176,115,121,50,45,127,4,150,254,138,236,205,93,
        222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180
    };

    // Fade function as defined by Ken Perlin. This eases coordinate values
    // so that they will "ease" towards integral values. This ends up smoothing the final output.
    inline float fade(float t) {
        return t * t * t * (t * (t * 6 - 15) + 10);
    }

    // Linear interpolation function.
    inline float lerp(float t, float a, float b) {
        return a + t * (b - a);
    }

    // Gradient function. Convert lower 4 bits of hash code into 12 gradient directions.
    inline float grad(int hash, float x, float y) {
        int h = hash & 7;      // Convert low 3 bits of hash into 8 directions
        float u = h < 4 ? x : y;
        float v = h < 4 ? y : x;
        return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
    }


    // 2D Perlin noise. Returns a noise value in roughly the range [-1, 1].
    inline float noise(float x, float y) {
        int X = static_cast<int>(std::floor(x)) & 255;
        int Y = static_cast<int>(std::floor(y)) & 255;

        x = x - std::floor(x);
        y = y - std::floor(y);

        float u = fade(x);
        float v = fade(y);

        int A = permutation[X] + Y;
        int B = permutation[(X + 1) & 255] + Y;

        float res = lerp(v,
                         lerp(u, grad(permutation[A & 255], x, y),
                                 grad(permutation[B & 255], x - 1, y)),
                         lerp(u, grad(permutation[(A + 1) & 255], x, y - 1),
                                 grad(permutation[(B + 1) & 255], x - 1, y - 1)));
        return res;
    }

        // ------------------------------------------------------------
    // Helper: Perlin on a sphere using only 2‑D noise
    //   dir = unit vector (x,y,z) on the globe
    //   freq = features per Earth diameter   (~4–10 is a good start)
    // ------------------------------------------------------------
    inline float perlinOnSphere2D(const std::tuple<float,float,float>& dir, float freq)
    {
        // Sample noise in the three coordinate planes
        float nXY = noise(std::get<0>(dir) * freq, std::get<1>(dir) * freq);
        float nYZ = noise(std::get<1>(dir) * freq, std::get<2>(dir) * freq);
        float nZX = noise(std::get<2>(dir) * freq, std::get<0>(dir)* freq);

        // Simple average keeps amplitude in ‑1 … +1
        return (nXY + nYZ + nZX) * (1.0f / 3.0f);
    }
} // namespace Perlin

#endif // QTPLANET_PERLIN_H