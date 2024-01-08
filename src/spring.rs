// Simple spring-and-mass simulation
use crate::prelude::*;

#[derive(Clone)]
pub struct StaticSpringAndMass {
	k: f64,
	mass: f64
}

impl StaticDifferentiator<2> for StaticSpringAndMass {
	fn new() -> Self {
		Self {
			k: 1.0,
			mass: 1.0
		}
	}
	fn differentiate(&mut self, state: &GenericVector<2>) -> NDimensionalDerivative<2> {
		// In this case there will be 2 pot variables: x and and dx
		// Force = -kx
		let force = -self.k * state.0[0];
		// Acc = force / mass
		let acc = force / self.mass;
		// Done
		NDimensionalDerivative(GenericVector([
			state.0[1],
			acc
		]))
	}
}