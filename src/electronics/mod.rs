// For electrical simulation
use std::collections::HashMap;

use crate::prelude::*;

pub mod components;
use components::*;


trait Component {
	fn state_representation_vec_size(&self) -> usize;
	fn initial_state(&self) -> VDyn;
	fn voltage_on_other_nodes(&self, node: usize, voltage: Float) -> HashMap<usize, Float>;
	fn apparent_resistance(&self, node1: usize, node2: usize) -> Float;
}

struct ComponentStatic {
	nodes: Vec<Node>
}

#[derive(Clone)]
struct Node {
	id: u32
}

struct CircuitStatic {
	nodes: Vec<Node>,
	components: Vec<Box<dyn Component>>
}

struct Circuit {
	static_: CircuitStatic,
	initial_state: VDyn
}

impl StaticDifferentiator for Circuit {
	fn state_representation_vec_size(&self) -> usize {
		// Number of nodes (voltage on each node) + component states
		let mut sum = self.static_.nodes.len();
		for c in &self.static_.components {
			sum += c.state_representation_vec_size();
		}
		// Done
		sum
	}
	fn begining_state(&self) -> VDyn {
		self.initial_state.clone()
	}
	fn differentiate(&self, state: &VDyn) -> NDimensionalDerivative {
		NDimensionalDerivative(VDyn::zeros(self.state_representation_vec_size()))// TODO
	}
}