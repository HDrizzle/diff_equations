// Soft-body 2D physics simulation
use std::collections::HashMap;
use crate::prelude::*;

#[derive(Clone)]
pub struct SoftBodySettings {
	precision: Float,// Ideal distance between nodes
	node_mass: Float,
	spring_k: Float,// Force-units / precision-units (NOT world-units)
	spring_damping: Float,// Force / velocity
	gravity: Float,// world-units/sec^2
	ground_k: Float,
}

impl Default for SoftBodySettings {
	fn default() -> Self {
		Self {
			precision: 1.0,
			node_mass: 1.0,
			spring_k: 10.0,
			spring_damping: 1.0,
			gravity: -9.81,
			ground_k: 100.0
		}
	}
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
	fn apply_iso(&self, state: &mut VDyn, iso: Iso2) {
		todo!()
	}
	pub fn get_node_pos_and_vel_from_state_vec(node_i: usize, state: &VDyn) -> (V2, V2) {
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
	pub fn set_node_vel_and_acc(node_i: usize, vel: V2, acc: V2, derivative: &mut NDimensionalDerivative) {
		// state has pos and vel; therefore the corresponding derivatives of those will be vel and acc
		Self::generic_set_state(node_i, vel, acc, &mut derivative.0);
	}
	fn generic_set_state(node_i: usize, v1: V2, v2: V2, state: &mut VDyn) {
		let start = node_i * 4;
		state[start    ] = v1.x;
		state[start + 1] = v1.y;
		state[start + 2] = v2.x;
		state[start + 3] = v2.y;
	}
	fn spring_force(&self, length: Float) -> Float {
		// Ideal length is 1. + is tension, - is compression
		let length_corrected = length - 1.0;
		length_corrected * self.settings.spring_k
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
				let absolute_grid_pos = node_grid_pos + grid_offset;
				if grid_offset != IntV2::zeros() && absolute_grid_pos.x >= 0 && absolute_grid_pos.x < self.bounding_box.x && absolute_grid_pos.y >= 0 && absolute_grid_pos.y < self.bounding_box.y {
					if self.does_node_exist_at_grid_pos(&absolute_grid_pos) {
						// Get "ideal" spring length based on relative grid position
						let grid_offset_float = intv2_to_v2(grid_offset);
						let ideal_length = grid_offset_float.magnitude() * self.settings.precision;
						// Get actual offset
						let (pos1, vel1) = Self::get_node_pos_and_vel_from_state_vec(node_i, state);
						let (pos2, vel2) = Self::get_node_pos_and_vel_from_state_vec(self.get_node_i_from_grid_pos(&absolute_grid_pos).expect(&format!("failed to get node index at grid position {:?}", absolute_grid_pos)), state);
						//dbg!(offset);
						// Finally get force
						net_force += self.force_between_nodes(ideal_length, pos2 - pos1, vel2 - vel1);
						//dbg!(net_force);
						// In case Y < 0 (touching ground)
						if pos1.y < 0.0 {
							net_force -= V2::new(0.0, pos1.y * self.settings.ground_k);
						}
						assert!(net_force.x.is_finite());
						assert!(net_force.y.is_finite());
					}
				}
			}
		}
		// Done
		net_force
	}
	fn force_between_nodes(&self, ideal_length: Float, rel_pos: V2, rel_vel: V2) -> V2 {
		(rel_pos / rel_pos.magnitude()) * (
			self.spring_force(rel_pos.magnitude() / ideal_length)// Static spring force
			+ 0.0// Damping, TODO: project rel_vel onto rel_pos and multiply magitude by self.settings.spring_damping
		)
	}
	fn does_node_exist_at_grid_pos(&self, grid_pos: &IntV2) -> bool {
		self.fill[grid_pos.x as usize][grid_pos.y as usize]
	}
	pub fn render_image(&self, state: &VDyn, image: &mut RgbImage, fill_color: Rgb<u8>, translater: &ImagePosTranslater) {
		let mut node_i: usize = 0;
		for x in 0..self.bounding_box.x {
			for y in 0..self.bounding_box.y {
				let grid_pos = IntV2::new(x, y);
				let is_node: bool = self.does_node_exist_at_grid_pos(&grid_pos);
				if is_node {
					let (pos, _) = Self::get_node_pos_and_vel_from_state_vec(node_i, state);
					match translater.world_to_px(pos) {
						Some(px_pos) => image.put_pixel(px_pos.x, px_pos.y, fill_color),
						None => {}
					}
					node_i += 1;
				}
			}
		}
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
		for x in 0..self.bounding_box.x {
			for y in 0..self.bounding_box.y {
				let grid_pos = IntV2::new(x, y);
				let is_node: bool = self.does_node_exist_at_grid_pos(&grid_pos);
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
					let acc: V2 = (force / self.settings.node_mass) + V2::new(0.0, self.settings.gravity);
					let (_, vel) = Self::get_node_pos_and_vel_from_state_vec(node_i, state);
					Self::set_node_vel_and_acc(node_i, vel, acc, &mut derivative);
					node_i += 1;
				}
			}
		}
		assert_vec_is_finite(&derivative.0).unwrap();
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

pub fn test() {
	// Use this command to generate video: $ ffmpeg -framerate 30 -i soft_body_render_%d.png -vcodec libx264 -r 30 -pix_fmt yuv420p output.mp4
	let body = SoftBody::from_bb_inclusion_func(
		IntV2::new(10, 10),
		|_| {true},
		SoftBodySettings {
			precision: 1.0,
			node_mass: 1.0,
			spring_k: 100.0,
			spring_damping: 1.0,
			gravity: -1.0,
			ground_k: 100.0
		}
	);
	let num_iterations: usize = 200;
	let dt = 0.02;
	let background = Rgb([0; 3]);
	let fill_color = Rgb([255; 3]);
	let image_size = ImgV2::new(500, 500);
	let translater = ImagePosTranslater {
		scale: 10.0,
		origin: ImgV2::zeros(),
		image_size
	};
	let mut stepper = Stepper::new(body, 10000.0, dt);
	// Run
	for i in 0..num_iterations {
		// Step
		stepper.step();
		// Render image
		let mut image: RgbImage = ImageBuffer::from_pixel(image_size.x, image_size.y, background);
		stepper.differentiator.render_image(&stepper.state, &mut image, fill_color, &translater);
		// Save image
		image.save(&format!("{}soft_body_render_{}.png", MEDIA_DIR, i));
	}
}

#[cfg(test)]
mod tests {
    use super::*;
	fn soft_body_settings() -> SoftBodySettings {
		SoftBodySettings {
			precision: 1.0,
			node_mass: 1.0,
			spring_k: 15.0,
			spring_damping: 1.0,
			gravity: -9.81,
			ground_k: 100.0
		}
	}
	#[test]
	fn gravity() {
		let settings = soft_body_settings();
		let body = SoftBody::from_bb_inclusion_func(IntV2::new(1, 1), |_| {true}, settings.clone());
		let mut stepper = Stepper::new(body, 10000.0, 1.0);
		stepper.step();
		assert_eq!(SoftBody::get_node_pos_and_vel_from_state_vec(0, &stepper.state).1, V2::new(0.0, settings.gravity));
	}
	#[test]
	fn force() {
		let settings = soft_body_settings();
		let body = SoftBody::from_bb_inclusion_func(IntV2::new(2, 1), |_| {true}, settings.clone());
		let mut state = body.begining_state();
		assert_eq!(body.state_representation_vec_size(), 8);
		assert_eq!(SoftBody::get_node_pos_and_vel_from_state_vec(0, &state), (V2::zeros(), V2::zeros()));
		assert_eq!(SoftBody::get_node_pos_and_vel_from_state_vec(1, &state), (V2::new(1.0, 0.0), V2::zeros()));
		// 1
		// Change 2nd node to be 2 units away from the first one
		SoftBody::generic_set_state(1, V2::new(2.0, 0.0), V2::zeros(), &mut state);
		// Calculate force
		assert_eq!(body.node_net_force(1, IntV2::new(1, 0), &state), V2::new(-15.0, 0.0));
		// 2
		// Change 2nd node to be 2 units away from the first one
		SoftBody::generic_set_state(1, V2::new(0.5, 0.0), V2::zeros(), &mut state);
		// Calculate force
		assert_eq!(body.node_net_force(1, IntV2::new(1, 0), &state), V2::new(5.0, 0.0));
	}
}