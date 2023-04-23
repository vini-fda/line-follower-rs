pub trait Float
where
    Self: num::Float + std::fmt::Display + std::fmt::Debug + nalgebra::RealField + 'static,
{
}
// implement for f32 and f64
impl Float for f32 {}
impl Float for f64 {}
