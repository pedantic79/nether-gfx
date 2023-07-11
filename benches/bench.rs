#![feature(portable_simd)]
use core::simd::f32x4;
use criterion::{criterion_group, criterion_main, Criterion};
use nether_gfx::fb::{FrameBuffer, Vertex};

fn helper(fb: &FrameBuffer, vert0: Vertex, vert1: Vertex, vert2: Vertex) {
    for mut tile in fb.tiles() {
        tile.draw_triangle(vert2, vert1, vert0)
    }
}

fn draw_triangles(c: &mut Criterion) {
    let vert0 = Vertex {
        proj: f32x4::from_array([
            6.0,
            256.0 + 500.0 / ((1.0 + 3.0f32.sqrt()) / 2.0) / 2.0,
            0.5,
            1.0,
        ]),
        color: f32x4::from_array([1.0, 0.0, 0.0, 1.0]),
    };
    let vert1 = Vertex {
        proj: f32x4::from_array([
            506.0,
            256.0 + 500.0 / ((1.0 + 3.0f32.sqrt()) / 2.0) / 2.0,
            0.5,
            1.0,
        ]),
        color: f32x4::from_array([0.0, 0.0, 1.0, 1.0]),
    };
    let vert2 = Vertex {
        proj: f32x4::from_array([
            256.0,
            256.0 - 500.0 / ((1.0 + 3.0f32.sqrt()) / 2.0) / 2.0,
            0.5,
            1.0,
        ]),
        color: f32x4::from_array([0.0, 1.0, 0.0, 1.0]),
    };
    let fb = FrameBuffer::new();

    c.bench_function("draw_triangles", |b| {
        b.iter(|| helper(&fb, vert0, vert1, vert2));
    });
}

criterion_group!(benches, draw_triangles);
criterion_main!(benches);
