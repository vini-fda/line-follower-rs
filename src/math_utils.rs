use num::Float;

pub struct FloatRange<F: Float> {
    pub start: F,
    pub end: F,
    pub step: F,
    curr: F,
}

impl<F> FloatRange<F>
where
    F: Float,
{
    pub fn new(start: F, end: F, step: F) -> Self {
        Self {
            start,
            end,
            step,
            curr: start,
        }
    }

    pub fn contains(&self, x: F) -> bool {
        x >= self.start && x <= self.end
    }
}

impl<F> Iterator for FloatRange<F>
where
    F: Float,
{
    type Item = F;
    fn next(&mut self) -> Option<Self::Item> {
        self.curr = self.curr + self.step;
        if self.curr > self.end {
            None
        } else {
            Some(self.curr)
        }
    }
}

#[inline(always)]
pub fn lattice_points<F: Float>(x_0: F, x_min: F, x_max: F, dx: F) -> FloatRange<F> {
    let y_min = x_0 + ((x_min - x_0) / dx).floor() * dx;
    let y_max = x_0 + ((x_max - x_0) / dx).ceil() * dx;
    FloatRange::new(y_min, y_max, dx)
}

#[inline(always)]
pub fn distance<F: Float>(x_0: F, y_0: F, x_1: F, y_1: F) -> F {
    ((x_0 - x_1).powi(2) + (y_0 - y_1).powi(2)).sqrt()
}

#[inline(always)]
pub fn dot_product<F: Float>(x_0: F, y_0: F, x_1: F, y_1: F) -> F {
    x_0 * x_1 + y_0 * y_1
}

#[inline(always)]
pub fn cross_product<F: Float>(x_0: F, y_0: F, x_1: F, y_1: F) -> F {
    // if going from (x_0, y_0) to (x_1, y_1) has a clockwise (<0) or counterclockwise(>0) orientation
    // this is equivalent to the determinant of the matrix [[x_0, y_0], [x_1, y_1]]
    x_0 * y_1 - x_1 * y_0
}

#[inline(always)]
pub fn sigmoid<F: Float>(x: F) -> F {
    F::one() / (F::one() + (-x).exp())
}