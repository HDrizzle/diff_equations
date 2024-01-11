// Inverse-square law
use crate::prelude::*;

pub mod soft_body;

#[derive(Clone)]
pub struct StaticSpringAndMass {
	pub k: Float,
	pub mass: Float
}

impl StaticDifferentiator for StaticSpringAndMass {
	type AuxData = ();
	fn state_representation_vec_size(&self) -> usize {
		2
	}
	fn initial_state(&self) -> (VDyn, ()) {
		(VDyn::from_vec(vec![0.0, 1.0]), ())
	}
	fn differentiate(&self, state: &VDyn, _: &mut ()) -> NDimensionalDerivative {
		// In this case there will be 2 plot variables: x and and dx
		// Force = -kx
		let force = -self.k * state[0];
		// Acc = force / mass
		let acc = force / self.mass;
		// Done
		NDimensionalDerivative(VDyn::from_vec(vec![
				state[1],
				acc
			]
		))
	}
}

impl Default for StaticSpringAndMass {
	fn default() -> Self {
		Self {
			k: 1.0,
			mass: 1.0
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	#[test]
	fn differentiation() {
		let mut d = StaticSpringAndMass{
			k: 1.0,
			mass: 1.0
		};
		let state = VDyn::from_vec(vec![0.0, 1.0]);
		assert_eq!(
			d.differentiate(&state, &mut ()),
			NDimensionalDerivative(VDyn::from_vec(vec![1.0, 0.0]))
		);
	}
}