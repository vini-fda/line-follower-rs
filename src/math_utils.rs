use num::Float;

pub struct FloatRange<F: Float> {
    pub start: F,
    pub end: F,
    pub step: F,
    curr: F,
}

impl<F> FloatRange<F> where F: Float {
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

impl<F> Iterator for FloatRange<F> where F: Float {
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