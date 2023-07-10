I'm trying to benchmark a tiled software rasterizer that I made, but am getting weird results, and the lack of proper documentation for `test::bench` doesn't help either.

My rasterizer produces an iterator that in turn produces tiles, which are small chunks of memory representing rectangular areas in the final image intended to maximize cache hits, so in order to draw a complete image I have to iterate over all the tiles and tell them to draw all the triangles in a command queue. The problem is that when I run the loop inside `Bench::iter` it seems to be optimized away as it only starts showing time results when I iterate a lot. For example on my system (an M1 Mac running MacOS Ventura), it only takes 4 seconds to iterate 10000000 times, and while it would indeed be nice if my rasterizer could produce 2500000 triangles per second on a single core, I don't think that's realistic even considering the performance gap because the same code only renders roughly 7000 triangles per second when running in parallel on all the 4 cores of a Raspberry Pi 4.

If you wish to try this out, nightly Rust is required, both because benchmarks are only available on that channel but also because I make heavy use of the unstable portable SIMD standard library module.

To make the project produce an image, type:

    cargo +nightly run triangle.png

To run the benchmark, type:

    cargo +nightly bench
