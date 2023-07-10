#![feature(test)]
#![feature(portable_simd)]

mod fb;

use std::simd::f32x4;
use std::path::Path;
use std::env::args;
use fb::{FrameBuffer, Vertex};

fn main() {
    let args = args().collect::<Vec<_>>();
    assert!(args.len() == 2, "Usage: {}, <file>", args[0]);
    let path = Path::new(&args[1]);
    let vert0 = Vertex {
        proj: f32x4::from_array([6.0, 256.0 + 500.0 / ((1.0 + 3.0f32.sqrt()) / 2.0) / 2.0, 0.5, 0.5]),
        color: f32x4::from_array([1.0, 0.0, 0.0, 1.0]),
    };
    let vert1 = Vertex {
        proj: f32x4::from_array([506.0, 256.0 + 500.0 / ((1.0 + 3.0f32.sqrt()) / 2.0) / 2.0, 0.5, 0.5]),
        color: f32x4::from_array([0.0, 0.0, 1.0, 1.0]),
    };
    let vert2 = Vertex {
        proj: f32x4::from_array([256.0, 256.0 - 500.0 / ((1.0 + 3.0f32.sqrt()) / 2.0) / 2.0, 0.5, 0.5]),
        color: f32x4::from_array([0.0, 1.0, 0.0, 1.0]),
    };
    let square0 = Vertex {
        proj: f32x4::from_array([0.0, 0.0, 0.25, 0.25]),
        color: f32x4::from_array([1.0, 1.0, 1.0, 1.0]),
    };
    let square1 = Vertex {
        proj: f32x4::from_array([512.0, 0.0, 0.25, 0.25]),
        color: f32x4::from_array([1.0, 1.0, 1.0, 1.0]),
    };
    let square2 = Vertex {
        proj: f32x4::from_array([0.0, 512.0, 0.25, 0.25]),
        color: f32x4::from_array([1.0, 1.0, 1.0, 1.0]),
    };
    let square3 = Vertex {
        proj: f32x4::from_array([512.0, 512.0, 0.25, 0.25]),
        color: f32x4::from_array([1.0, 1.0, 1.0, 1.0]),
    };
    let fb = FrameBuffer::new();
    for mut tile in fb.tiles() {
        tile.draw_triangle(vert2, vert1, vert0);
        tile.draw_triangle(square0, square1, square2);
        tile.draw_triangle(square2, square1, square3);
    }
    fb.dump(&path);
}
