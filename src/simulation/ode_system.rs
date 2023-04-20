pub type Vector<const N: usize> = nalgebra::SVector<f64, N>;

#[allow(dead_code)]
pub struct OdeSystem<F, const N: usize> 
where F: FnMut(f64, &Vector<N>) -> Vector<N> {
    pub t: f64,
    pub x: Vector<N>,
    pub f: F,
}

impl<F, const N: usize> OdeSystem<F, N> 
where F: FnMut(f64, &Vector<N>) -> Vector<N> {
    pub fn new(t: f64, x: Vector<N>, f: F) -> Self {
        Self { t, x, f }
    }
}