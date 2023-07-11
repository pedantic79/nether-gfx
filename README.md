This project contains a software rasterizer written in Rust that I need help optimizing.

Optimizations that I have already implemented:

* Render to 32x32 tiles of 4KB each (2KB 16-bit color and 2KB 16-bit depth) to maximize cache hits;
* Use SIMD to compute values for 4 pixels at once;
* Skip a triangle if its axis-aligned bounding box is completely outside the current tile's bounding box;
* Skip a triangle if at least one of its barycentric coordinates is negative on all 4 corners of the current tile;
* Compute the linear barycentric increments per pixel and use that information to avoid having to perform the edge test for every pixel;
* Skip a triangle if, by the time of shading, all the pixels have been invalidated.

At the moment the original version of this code exhausts all 4 cores of a Raspberry Pi 4 with just 7000 triangles per second, and this benchmark takes roughly 300 microseconds to produce a 512x512 frame with a rainbow triangle with perspective correction and depth testing on an M1 Mac, so to me the performance is really bad.

What I'm trying to understand is how old school games with true 3D software rasterizers performed so well even on old hardware like a Pentium 166Mhz without floating pointe SIMD or multiple cores. Optimization is a field that truly excites me, and I believe that cracking this problem will be extremely enriching.
