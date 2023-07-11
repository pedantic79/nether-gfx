//! Frame buffer rendering target.

use core::alloc::Layout;
use core::iter::Iterator;
use core::simd::{f32x4, u16x4, u32x4, SimdFloat, SimdPartialEq, SimdPartialOrd, SimdUint};
use core::sync::atomic::{AtomicU64, Ordering};

use std::alloc::{GlobalAlloc, System};
use std::fs::File;
use std::path::Path;
use std::io::BufWriter;
use png::*;

/// Maximum width or height of a tile.
const TILE_DIM_MAX: usize = 32;

/// Frame buffer.
pub struct FrameBuffer
{
    /// Frame buffer.
    buf: *mut u16,
    /// Image width.
    width: usize,
    /// Tile width.
    twidth: usize,
    /// Tile height.
    theight: usize,
    /// Tile count.
    tcount: usize,
    /// Id of the next tile to draw.
    tnext: AtomicU64,
    /// Finished tile counter.
    tfinished: AtomicU64,
}

/// Frame buffer iterator.
pub struct FrameBufferIterator<'a>
{
    /// Frame buffer being iterated.
    fb: &'a FrameBuffer,
    /// Frame being iterated.
    frame: u64,
}

/// Frame buffer tile.
pub struct Tile<'a>
{
    /// Frame buffer that this tile draws to.
    fb: &'a FrameBuffer,
    /// Origin column for this tile.
    col: usize,
    /// Origin row for this tile.
    row: usize,
    /// X control point coordinates.
    xctl: f32x4,
    /// Y control point coordinates.
    yctl: f32x4,
    /// Tile's color buffer.
    cb: Buffer,
    /// Tile's depth buffer.
    db: Buffer,
}

/// Vertex.
#[derive(Clone, Copy, Debug)]
pub struct Vertex
{
    /// Projected position.
    pub proj: f32x4,
    /// RGBA color.
    pub color: f32x4,
}

/// Tile buffer.
#[repr(align(0x40), C)]
#[derive(Debug)]
struct Buffer([u16; TILE_DIM_MAX * TILE_DIM_MAX]);

impl FrameBuffer
{
    /// Creates and initializes a new frame buffer.
    ///
    /// Returns the newly created frame buffer.
    pub fn new() -> Self
    {
        Self { buf: unsafe {System.alloc_zeroed(Layout::new::<[u16; 512 * 512]>()).cast()},
               width: 512,
               twidth: TILE_DIM_MAX,
               theight: TILE_DIM_MAX,
               tcount: 512 * 512 / (TILE_DIM_MAX * TILE_DIM_MAX),
               tnext: AtomicU64::new(0),
               tfinished: AtomicU64::new(0) }
    }

    /// Creates an iterator of tiles awaiting to be drawn.
    ///
    /// Returns the newly created iterator.
    pub fn tiles(&self) -> FrameBufferIterator
    {
        FrameBufferIterator::new(self)
    }
    
    pub fn dump(&self, path: &Path) {
        let output = File::create(path).unwrap();
        let input = unsafe {(*self.buf.cast::<[u16; 512 * 512]>()).iter().map(Self::convert).flatten().collect::<Vec<_>>()};
        let writer = BufWriter::new(output);
        let mut png = Encoder::new(writer, 512, 512);
        png.set_color(ColorType::Rgb);
        let mut writer = png.write_header().unwrap();
        writer.write_image_data(&input).unwrap();
    }

    fn convert(rgb565: &u16) -> [u8; 3] {
        let red = (rgb565 & 0xf800) >> 8;
        let red = red | (red >> 5);
        let green = (rgb565 & 0x7e0) >> 3;
        let green = green | (green >> 6);
        let blue = (rgb565 & 0x1f) << 3;
        let blue = blue | (blue >> 5);
        [red as _, green as _, blue as _]
    }
}

impl<'a> FrameBufferIterator<'a>
{
    /// Creates and initializes a new iterator over the tiles of a frame buffer.
    ///
    /// * `fb`: Frame buffer that this iterator borrows tiles from.
    ///
    /// Returns the newly created iterator.
    fn new(fb: &'a FrameBuffer) -> Self
    {
        Self { fb, frame: fb.tnext.load(Ordering::Relaxed) / fb.tcount as u64 }
    }
}

impl<'a> Iterator for FrameBufferIterator<'a>
{
    type Item = Tile<'a>;

    fn next(&mut self) -> Option<Tile<'a>>
    {
        let tnext = loop {
            let tnext = self.fb.tnext.load(Ordering::Relaxed);
            if tnext / self.fb.tcount as u64 != self.frame {
                return None;
            };
            if self.fb
                   .tnext
                   .compare_exchange(tnext, tnext + 1, Ordering::Relaxed, Ordering::Relaxed)
                   .is_ok()
            {
                break tnext;
            }
        };
        Some(Tile::new(self.fb, tnext))
    }
}

impl<'a> Tile<'a>
{
    /// Creates and initializes a new tile.
    ///
    /// * `fb`: Frame buffer that this tile represents.
    /// * `id`: ID of the tile.
    ///
    /// Returns the newly created tile.
    fn new(fb: &'a FrameBuffer, id: u64) -> Self
    {
        let pos = id as usize % fb.tcount;
        let col = pos * fb.twidth % fb.width;
        let row = pos * fb.twidth / fb.width * fb.theight;
        let xdist = fb.twidth as f32;
        let ydist = fb.theight as f32;
        let xmin = col as f32;
        let ymin = row as f32;
        let xmax = xmin + xdist;
        let ymax = ymin + ydist;
        let xctl = f32x4::from([xmin, xmax, xmin, xmax]);
        let yctl = f32x4::from([ymin, ymin, ymax, ymax]);
        let cb = Buffer([0; TILE_DIM_MAX * TILE_DIM_MAX]);
        let db = Buffer([0; TILE_DIM_MAX * TILE_DIM_MAX]);
        Self { fb, col, row, xctl, yctl, cb, db }
    }

    /// Draws a triangle to the tile.
    ///
    /// * `vert0`: First vertex.
    /// * `vert1`: Second vertex.
    /// * `vert2`: Third vertex.
    pub fn draw_triangle(&mut self, vert0: Vertex, vert1: Vertex, vert2: Vertex)
    {
        let ptx = self.xctl;
        let pty = self.yctl;
        // Check whether the triangle's axis aligned bounding box overlaps this tile.
        let tri_max = vert0.proj.simd_max(vert1.proj).simd_max(vert2.proj);
        let tile_min = f32x4::from([ptx[0], pty[0], 0.0, 0.0]);
        if tri_max.simd_lt(tile_min).any() {
            // Triangle is guaranteed to be completely outside this tile.
            return;
        }
        let tri_min = vert0.proj.simd_min(vert1.proj).simd_min(vert2.proj);
        let tile_max = f32x4::from([ptx[3], pty[3], 1.0, f32::INFINITY]);
        if tri_min.simd_gt(tile_max).any() {
            // Triangle is completely outside this tile.
            return;
        }
        // Compute the linear barycentric coordinates of the fragments at the tile's
        // corners.
        let x0 = f32x4::splat(vert0.proj[0]) - ptx;
        let y0 = f32x4::splat(vert0.proj[1]) - pty;
        let x1 = f32x4::splat(vert1.proj[0]) - ptx;
        let y1 = f32x4::splat(vert1.proj[1]) - pty;
        let x2 = f32x4::splat(vert2.proj[0]) - ptx;
        let y2 = f32x4::splat(vert2.proj[1]) - pty;
        let area0 = x1 * y2 - x2 * y1;
        if area0.reduce_max() < 0.0 {
            // The whole triangle is outside the tile.
            return;
        }
        let area1 = x2 * y0 - x0 * y2;
        if area1.reduce_max() < 0.0 {
            // The whole triangle is outside the tile.
            return;
        }
        let area2 = x0 * y1 - x1 * y0;
        if area2.reduce_max() < 0.0 {
            // The whole triangle is outside the tile.
            return;
        }
        let total_recip = (area0 + area1 + area2).recip();
        let mut bary0 = area0 * total_recip;
        let mut bary1 = area1 * total_recip;
        let mut bary2 = area2 * total_recip;
        // Compute the linear barycentric coordinate increments.
        let twidth = self.fb.twidth;
        let theight = self.fb.theight;
        let xdiv = (twidth as f32).recip();
        let ydiv = (theight as f32).recip();
        let hinc0 = (bary0[1] - bary0[0]) * xdiv;
        let vinc0 = (bary0[2] - bary0[0]) * ydiv;
        bary0[0] += hinc0 * 0.5 + vinc0 * 0.5;
        bary0[1] = bary0[0] + hinc0;
        bary0[2] = bary0[1] + hinc0;
        bary0[3] = bary0[2] + hinc0;
        let hinc0 = f32x4::splat(hinc0 * 4.0);
        let vinc0 = f32x4::splat(vinc0);
        let hinc1 = (bary1[1] - bary1[0]) * xdiv;
        let vinc1 = (bary1[2] - bary1[0]) * ydiv;
        bary1[0] += hinc1 * 0.5 + vinc1 * 0.5;
        bary1[1] = bary1[0] + hinc1;
        bary1[2] = bary1[1] + hinc1;
        bary1[3] = bary1[2] + hinc1;
        let hinc1 = f32x4::splat(hinc1 * 4.0);
        let vinc1 = f32x4::splat(vinc1);
        let hinc2 = (bary2[1] - bary2[0]) * xdiv;
        let vinc2 = (bary2[2] - bary2[0]) * ydiv;
        bary2[0] += hinc2 * 0.5 + vinc2 * 0.5;
        bary2[1] = bary2[0] + hinc2;
        bary2[2] = bary2[1] + hinc2;
        bary2[3] = bary2[2] + hinc2;
        let hinc2 = f32x4::splat(hinc2 * 4.0);
        let vinc2 = f32x4::splat(vinc2);
        // Loop over all the pixels in the tile in groups of 4, and shade those that
        // belong to the triangle.
        let mut vbary0 = bary0;
        let mut vbary1 = bary1;
        let mut vbary2 = bary2;
        let zero = f32x4::splat(0.0);
        let one = f32x4::splat(1.0);
        let dxm = u32x4::splat(0x3F800000);
        let dxb = u32x4::splat(0x30000000);
        let dmm = u32x4::splat(0x7FF000);
        let ds = u32x4::splat(12);
        let rbmul = f32x4::splat(31.5);
        let gmul = f32x4::splat(63.5);
        let rshift = u32x4::splat(11);
        let gshift = u32x4::splat(5);
        for trow in 0 .. theight {
            let mut hbary0 = vbary0;
            let mut hbary1 = vbary1;
            let mut hbary2 = vbary2;
            for tcol in (0 .. twidth).step_by(4) {
                let offset = trow * twidth + tcol;
                // Fill the triangle.
                let mut valid = hbary0.simd_gt(zero) & hbary1.simd_gt(zero) & hbary2.simd_gt(zero);
                // Also draw the top, top-right, right, and bottom-right edges.
                if (hbary0.simd_eq(zero) | hbary1.simd_eq(zero) | hbary2.simd_eq(zero)).any() {
                    valid |= hbary0.simd_eq(zero) & (hinc0.simd_lt(zero) | hinc0.simd_eq(zero) & vinc0.simd_lt(zero));
                    valid |= hbary1.simd_eq(zero) & (hinc1.simd_lt(zero) | hinc1.simd_eq(zero) & vinc1.simd_lt(zero));
                    valid |= hbary2.simd_eq(zero) & (hinc2.simd_lt(zero) | hinc2.simd_eq(zero) & vinc2.simd_lt(zero));
                }
                // Compute the perspective-correct barycentric coordinates.
                let w0 = f32x4::splat(vert0.proj[3]) * hbary0;
                let w1 = f32x4::splat(vert1.proj[3]) * hbary1;
                let w2 = f32x4::splat(vert2.proj[3]) * hbary2;
                let wp = (w0 + w1 + w2).recip();
                let bary0 = w0 * wp;
                let bary1 = w1 * wp;
                let bary2 = w2 * wp;
                // Compute the depth and exclude all fragments outside the range between the
                // values in the depth buffer and the near clipping plane.
                let db = unsafe { self.db.0.as_mut_ptr().add(offset).cast::<u16x4>() };
                let odepth = unsafe { db.read() };
                let z0 = f32x4::splat(vert0.proj[2]) * bary0;
                let z1 = f32x4::splat(vert1.proj[2]) * bary1;
                let z2 = f32x4::splat(vert2.proj[2]) * bary2;
                let z = z0 + z1 + z2;
                valid &= z.simd_le(one);
                let zb = z.to_bits().saturating_sub(dxb);
                let zx = (zb & dxm) >> ds;
                let zm = (zb & dmm) >> ds;
                let depth = (zx | zm).cast::<u16>();
                let mut valid = valid.cast::<i16>();
                valid &= depth.simd_gt(odepth);
                if valid.any() {
                    // Store the new depth values.
                    let depth = valid.select(depth, odepth);
                    unsafe { db.write(depth) };
                    // Apply shading.
                    let cb = unsafe { self.cb.0.as_mut_ptr().add(offset).cast::<u16x4>() };
                    let ocolor = unsafe { cb.read() };
                    let red0 = f32x4::splat(vert0.color[0]) * bary0;
                    let red1 = f32x4::splat(vert1.color[0]) * bary1;
                    let red2 = f32x4::splat(vert2.color[0]) * bary2;
                    let red = red0 + red1 + red2;
                    let red = red.simd_max(zero).simd_min(one);
                    let green0 = f32x4::splat(vert0.color[1]) * bary0;
                    let green1 = f32x4::splat(vert1.color[1]) * bary1;
                    let green2 = f32x4::splat(vert2.color[1]) * bary2;
                    let green = green0 + green1 + green2;
                    let green = green.simd_max(zero).simd_min(one);
                    let blue0 = f32x4::splat(vert0.color[2]) * bary0;
                    let blue1 = f32x4::splat(vert1.color[2]) * bary1;
                    let blue2 = f32x4::splat(vert2.color[2]) * bary2;
                    let blue = blue0 + blue1 + blue2;
                    let blue = blue.simd_max(zero).simd_min(one);
                    // Compute the RGB565 color values.
                    let red = (red * rbmul).cast::<u32>() << rshift;
                    let green = (green * gmul).cast::<u32>() << gshift;
                    let blue = (blue * rbmul).cast::<u32>();
                    let color = (red | green | blue).cast::<u16>();
                    let color = valid.select(color, ocolor);
                    unsafe { cb.write(color) };
                }
                hbary0 += hinc0;
                hbary1 += hinc1;
                hbary2 += hinc2;
            }
            vbary0 += vinc0;
            vbary1 += vinc1;
            vbary2 += vinc2;
        }
    }
}

impl<'a> Drop for Tile<'a>
{
    fn drop(&mut self)
    {
        let buf = unsafe { self.fb.buf.add(self.row * self.fb.width + self.col) };
        for trow in 0 .. self.fb.theight {
            unsafe {
                let buf = buf.add(trow * self.fb.width);
                let cb = self.cb.0.as_ptr().add(trow * self.fb.twidth);
                cb.copy_to_nonoverlapping(buf, self.fb.twidth);
            }
        }
        self.fb.tfinished.fetch_add(1, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    extern crate test;
    
    use test::bench::{Bencher, black_box};
    use super::*;
    
    #[bench]
    fn draw_triangles(bencher: &mut Bencher) {
        let vert0 = Vertex {
            proj: f32x4::from_array([6.0, 256.0 + 500.0 / ((1.0 + 3.0f32.sqrt()) / 2.0) / 2.0, 0.5, 1.0]),
            color: f32x4::from_array([1.0, 0.0, 0.0, 1.0]),
        };
        let vert1 = Vertex {
            proj: f32x4::from_array([506.0, 256.0 + 500.0 / ((1.0 + 3.0f32.sqrt()) / 2.0) / 2.0, 0.5, 1.0]),
            color: f32x4::from_array([0.0, 0.0, 1.0, 1.0]),
        };
        let vert2 = Vertex {
            proj: f32x4::from_array([256.0, 256.0 - 500.0 / ((1.0 + 3.0f32.sqrt()) / 2.0) / 2.0, 0.5, 1.0]),
            color: f32x4::from_array([0.0, 1.0, 0.0, 1.0]),
        };
        let fb = FrameBuffer::new();
        bencher.iter(|| black_box(fb.tiles().for_each(|mut tile| tile.draw_triangle(vert2, vert1, vert0))));
    }
}
