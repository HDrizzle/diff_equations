// Soft-body 2D physics simulation
use std::collections::HashMap;
use crate::prelude::*;

pub struct SoftBodySettings {
	precision: Float,// Ideal distance between nodes
	node_mass: Float,
	spring_k: Float,// Force-units / precision-units (NOT world-units)
	gravity: Float// world-units/sec^2
}

pub struct SoftBody {// Simulates a grid of "nodes" (points with mass) connected to each other with "springs"
	settings: SoftBodySettings,
	bounding_box: IntV2,
	fill: Vec<Vec<bool>>,// [[value; Y]; X], whether a given location on the grid is occupied
	num_nodes: usize,
	node_index_lookup: HashMap<IntV2, usize>
}

impl SoftBody {
	pub fn from_bb_inclusion_func<F>(bb: IntV2, func: F, settings: SoftBodySettings) -> Self
	where F: Fn(IntV2) -> bool {
		let mut fill = Vec::<Vec<bool>>::new();
		let mut node_index_lookup = HashMap::<IntV2, usize>::new();
		let mut num_nodes: usize = 0;
		for x in 0..bb.x {
			let mut col = Vec::<bool>::new();
			for y in 0..bb.y {
				let is_node = func(IntV2::new(x, y));
				if is_node {
					node_index_lookup.insert(IntV2::new(x, y), num_nodes);
					num_nodes += 1;
				}
				col.push(is_node);
			}
			fill.push(col);
		}
		// Done
		Self {
			settings,
			bounding_box: bb,
			fill,
			num_nodes,
			node_index_lookup
		}
	}
	fn get_node_pos_and_vel_from_state_vec(&self, node_i: usize, state: &VDyn) -> (V2, V2) {
		// (position, velocity)
		let start = node_i * 4;
		(
			V2::new(
				state[start],
				state[start + 1]
			),
			V2::new(
				state[start + 2],
				state[start + 3]
			)
		)
	}
	fn set_node_vel_and_acc(&self, node_i: usize, vel: V2, acc: V2, derivative: &mut NDimensionalDerivative) {
		// state has pos and vel; therefore the corresponding derivatives of those will be vel and acc
		let start = node_i * 4;
		// Derivative pos = vel
		derivative.0[start] = vel.x;
		derivative.0[start + 1] = vel.y;
		// Derivative vel = acc
		derivative.0[start + 2] = acc.x;
		derivative.0[start + 3] = acc.y;
	}
	fn spring_force(&self, length: Float) -> Float {
		// Ideal length is 1. + is tension, - is compression
		let length_corrected = length - 1.0;
		length * self.settings.spring_k
	}
	fn get_node_i_from_grid_pos(&self, grid_pos: &IntV2) -> Option<usize> {
		match self.node_index_lookup.get(grid_pos) {
			Some(i) => Some(*i),
			None => None
		}
	}
	fn node_net_force(&self, node_i: usize, node_grid_pos: IntV2, state: &VDyn) -> V2 {
		let mut net_force = V2::zeros();
		for x_offset in -1..2 {
			for y_offset in -1..2 {
				let grid_offset = IntV2::new(x_offset, y_offset);
				if grid_offset != IntV2::zeros() && grid_offset.x >= 0 && grid_offset.x < self.bounding_box.x && grid_offset.y >= 0 && grid_offset.y < self.bounding_box.y {
					let absolute_grid_pos = node_grid_pos + node_grid_pos;
					if self.does_node_exist_at_grid_pos(&absolute_grid_pos) {
						// Get "ideal" spring length based on relative grid position
						let grid_offset_float = intv2_to_v2(grid_offset);
						let ideal_length = grid_offset_float.magnitude() * self.settings.precision;
						// Get actual offset
						let offset =
							self.get_node_pos_and_vel_from_state_vec(self.get_node_i_from_grid_pos(&absolute_grid_pos).expect(&format!("failed to get node index at grid position {:?}", absolute_grid_pos)), state).0 -
							self.get_node_pos_and_vel_from_state_vec(node_i, state).0;
						// Finally get force
						net_force += offset.normalize() * self.spring_force(offset.magnitude() / ideal_length);
					}
				}
			}
		}
		// Done
		net_force
	}
	fn does_node_exist_at_grid_pos(&self, grid_pos: &IntV2) -> bool {
		self.fill[grid_pos.x as usize][grid_pos.y as usize]
	}
}

impl StaticDifferentiator for SoftBody {
	/* The vector state representation will have the following format for each node index `i`
	i*4 + 0: X position
	i*4 + 1: Y position
	i*4 + 2: X velocity
	i*4 + 3: Y velocity
	*/
	fn state_representation_vec_size(&self) -> usize {
		// (Number of nodes * 2 dimensions) * 2 (positions and velocities)
		self.num_nodes * 4
	}
	fn begining_state(&self) -> VDyn {
		let mut out = VDyn::from_vec(vec![0.0; self.state_representation_vec_size()]);
		let mut node_i: usize = 0;
		for x in 0..self.bounding_box.x as usize {
			for y in 0..self.bounding_box.y as usize {
				let is_node: bool = self.fill[x][y];
				if is_node {
					// Set node position and velocity
					let start = node_i * 4;
					// Position
					out[start] = (x as Float) * self.settings.precision;
					out[start + 1] = (y as Float) * self.settings.precision;
					// Velocity
					out[start + 2] = 0.0;
					out[start + 3] = 0.0;
					// Don't forgot to
					node_i += 1;
				}
			}
		}
		// Safety check
		assert_eq!(node_i, self.num_nodes, "When iterating over `self.fill`, the number of occupied nodes counted ({}) was != to `self.num_nodes` ({})", node_i, self.num_nodes);
		// Done
		out
	}
	fn differentiate(&self, state: &VDyn) -> NDimensionalDerivative {
		let mut derivative = NDimensionalDerivative(VDyn::from_vec(vec![0.0; self.state_representation_vec_size()]));
		let mut node_i: usize = 0;
		for x in 0..self.bounding_box.x {
			for y in 0..self.bounding_box.y {
				let grid_pos = IntV2::new(x, y);
				let is_node: bool = self.does_node_exist_at_grid_pos(&grid_pos);
				if is_node {
					// Calculate acceleration
					let force: V2 = self.node_net_force(node_i, grid_pos, state);
					let acc: V2 = force / self.settings.node_mass;
					let (_, vel) = self.get_node_pos_and_vel_from_state_vec(node_i, state);
					self.set_node_vel_and_acc(node_i, vel, acc, &mut derivative);
					node_i += 1;
				}
			}
		}
		// Done
		derivative
	}
}

/*struct IteratedNode {
	x: usize,
	y: usize,
	i: usize
}

struct NodeIterator<'a> {
	body: &'a SoftBody,
	bb: IntV2,
	x: usize,
	y: usize,
	node_i: usize
}

impl<'a> Iterator for NodeIterator<'a> {
	type Item = IteratedNode;
	fn next(&mut self) -> Option<Self::Item> {
		
	}
}*/