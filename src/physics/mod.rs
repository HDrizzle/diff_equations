// Inverse-square law
use crate::prelude::*;

/*#[derive(Clone, Debug)]
pub struct ObjectStatic {
	mass: Float
}

#[derive(Clone, Debug)]
pub struct Object<const N: usize> {
	static_: ObjectStatic,
	pos: GenericVector<N>
}

#[derive(Clone, Debug)]
pub struct System<const N: usize> {
	objects: Vec<Object<N>>
}

impl<const N: usize> StaticDifferentiator<{ N * 2 }> for System<N> {
	fn differentiate(&mut self, state: &GenericVector<{N*2}>) -> NDimensionalDerivative<{N*2}> {
		
	}
}*/

#[derive(Clone)]
pub struct StaticSpringAndMass {
	pub k: Float,
	pub mass: Float
}

impl StaticDifferentiator for StaticSpringAndMass {
	fn beginning_state_size(&self) -> usize {
		2
	}
	fn begining_state(&self) -> VDyn {
		VDyn::from_vec(vec![0.0, 1.0])
	}
	fn differentiate(&mut self, state: &VDyn) -> NDimensionalDerivative {
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
			d.differentiate(&state),
			NDimensionalDerivative(VDyn::from_vec(vec![1.0, 0.0]))
		);
	}
}