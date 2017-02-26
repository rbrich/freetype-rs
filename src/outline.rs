use std::slice;
use std::marker::PhantomData;
use libc::{ c_short, c_char };
use { ffi, Vector };

#[derive(Copy, Clone)]
pub enum Curve {
    Line(Vector),
    Bezier2(Vector, Vector),
    Bezier3(Vector, Vector, Vector)
}

pub struct Outline<'a> {
    raw: &'a ffi::FT_Outline
}

impl<'a> Outline<'a> {
    pub unsafe fn from_raw(raw: &'a ffi::FT_Outline) -> Self {
        Outline {
            raw: raw
        }
    }

    pub fn points(&self) -> &'a [Vector] {
        unsafe {
            slice::from_raw_parts(self.raw.points, self.raw.n_points as usize)
        }
    }

    pub fn tags(&self) -> &'a [c_char] {
        unsafe {
            slice::from_raw_parts(self.raw.tags, self.raw.n_points as usize)
        }
    }

    pub fn contours(&self) -> &'a [c_short] {
        unsafe {
            slice::from_raw_parts(self.raw.contours, self.raw.n_contours as usize)
        }
    }

    pub fn contours_iter(&self) -> ContourIterator<'a> {
        unsafe {
            ContourIterator::from_raw(self.raw)
        }
    }
}

const TAG_MASK: c_char = 0x03;
const TAG_ONCURVE: c_char = 0x01;
const TAG_BEZIER2: c_char = 0x00;
const TAG_BEZIER3: c_char = 0x02;

#[inline]
fn middle_point(pt1: Vector, pt2: Vector) -> Vector {
    ffi::FT_Vector {
        x: (pt1.x + pt2.x) / 2,
        y: (pt1.y + pt2.y) / 2,
    }
}

pub struct CurveIterator<'a> {
    start_point: *const Vector,
    start_tag: *const c_char,
    idx: isize,
    length: isize,
    marker: PhantomData<&'a ()>
}

impl<'a> CurveIterator<'a> {
    pub unsafe fn from_raw(outline: &'a ffi::FT_Outline,
                               start_idx: isize,
                               end_idx: isize) -> Self {
        CurveIterator {
            start_point: outline.points.offset(start_idx),
            start_tag: outline.tags.offset(start_idx),
            // If the first tag is off-curve, start at -1 and return last or interpolated point as start()
            idx: if *outline.tags.offset(start_idx) & TAG_MASK == TAG_BEZIER2 { -1 } else { 0 },
            length: end_idx - start_idx + 1,
            marker: PhantomData
        }
    }

    pub fn start(&self) -> Vector {
        unsafe {
            // Check if starting point is off-curve, use last point in that case
            if *self.start_tag & TAG_MASK == TAG_BEZIER2 {
                match *self.start_tag.offset(self.length - 1) {
                    TAG_ONCURVE => *self.start_point.offset(self.length - 1),
                    // If last point is also off-curve, then interpolate
                    TAG_BEZIER2 => middle_point(*self.start_point.offset(self.length - 1),
                                                *self.start_point),
                    _ => panic!("Unexpected curve tag"),
                }
            } else {
                *self.start_point
            }
        }
    }

    // Retrieves the point at offset i from the current point. Note that contours implicitly repeat their
    // first point at the end.
    unsafe fn pt(&self, i: isize) -> Vector {
        if self.idx + i < self.length {
            *self.start_point.offset(self.idx + i)
        } else {
            *self.start_point
        }
    }

    unsafe fn tg(&self, i: isize) -> c_char {
        if self.idx + i < self.length {
            *self.start_tag.offset(self.idx + i)
        } else {
            *self.start_tag
        }
    }
}

impl<'a> Iterator for CurveIterator<'a> {
    type Item = Curve;

    fn next(&mut self) -> Option<Self::Item> {
        if self.idx >= self.length {
            None
        } else {
            unsafe {
                let (shift, curve) =
                match self.tg(1) {
                    TAG_ONCURVE => (1, Curve::Line(self.pt(1))),
                    TAG_BEZIER2 => {
                        // We are some kind of quadratic Bezier.
                        // Quadratic Bezier curves have a special treatment in TTF outlines:
                        // as an optimization, curves are often constructed from sequences
                        // of off-curve control points. In this case, there are implied on-curve
                        // points in between each pair of off-curve points.

                        // If we are at last point and checking first point (in circular fashion),
                        // and it's off-curve, then stop iterating. The last point was already
                        // reported by start() method.
                        if self.idx == self.length - 1 {
                            return None;
                        }

                        match self.tg(2) {
                            TAG_ONCURVE => (2, Curve::Bezier2(self.pt(1), self.pt(2))),
                            TAG_BEZIER2 => (1, Curve::Bezier2(self.pt(1), middle_point(self.pt(1), self.pt(2)))),
                            _ => panic!("Unexpected curve tag"),
                        }
                    },
                    TAG_BEZIER3 => {
                        debug_assert!(self.tg(2) == TAG_BEZIER3);
                        debug_assert!(self.tg(3) == TAG_ONCURVE);
                        (3, Curve::Bezier3(self.pt(1), self.pt(2), self.pt(3)))
                    },
                    _ => panic!("Unexpected curve tag"),
                };

                self.idx += shift;
                Some(curve)
            }
        }
    }
}

pub struct ContourIterator<'a> {
    outline: &'a ffi::FT_Outline,
    contour_start: c_short,
    contour_end_idx: *const c_short,
    last_end_idx: *const c_short
}

impl<'a> ContourIterator<'a> {
    pub unsafe fn from_raw(outline: &'a ffi::FT_Outline) -> Self {
        ContourIterator {
            outline: outline,
            contour_start: 0,
            contour_end_idx: outline.contours,
            last_end_idx: outline.contours.offset(outline.n_contours as isize)
        }
    }
}

impl<'a> Iterator for ContourIterator<'a> {
    type Item = CurveIterator<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.contour_end_idx >= self.last_end_idx {
            None
        } else {
            unsafe {
                let contour_end = *self.contour_end_idx;
                let curves = CurveIterator::from_raw(self.outline, self.contour_start as isize,
                                                     contour_end as isize);
                self.contour_start = contour_end + 1;
                self.contour_end_idx = self.contour_end_idx.offset(1);

                Some(curves)
            }
        }
    }
}
