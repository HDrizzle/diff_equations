// Simple spring-and-mass simulation
use crate::prelude::*;

#[derive(Clone)]
pub struct StaticSpringAndMass {
	k: Float,
	mass: Float
}

impl StaticDifferentiator<2> for StaticSpringAndMass {
	fn new() -> Self {
		Self {
			k: 1.0,
			mass: 1.0
		}
	}
	fn differentiate(&mut self, state: &GenericVector<2>) -> NDimensionalDerivative<2> {
		// In this case there will be 2 plot variables: x and and dx
		// Force = -kx
		let force = -self.k * state.0[0];
		// Acc = force / mass
		let acc = force / self.mass;
		// Done
		NDimensionalDerivative(GenericVector([
			state.0[1],
			acc
			/*state.0[1],
			-state.0[0]*/
		]))
	}
}

#[cfg(test)]
mod tests {
    use super::*;
	#[test]
	fn differentiation() {
		let mut d = StaticSpringAndMass::new();
		let state = GenericVector::<2>([0.0, 1.0]);
		assert_eq!(
			d.differentiate(&state),
			NDimensionalDerivative(GenericVector([1.0, 0.0]))
		);
	}
}