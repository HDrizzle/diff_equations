// Soft-body 2D physics simulation
use std::collections::HashMap;
use bevy::text;
use nalgebra::{UnitComplex, Translation, Dyn, SimdValue};
use approx::assert_relative_eq;
use image::{RgbaImage, GenericImageView};

use crate::prelude::*;

#[derive(Clone)]
pub struct SoftBodySettings {
	precision: Float,// Ideal distance between nodes
	node_mass: Float,
	spring_k: Float,// Force-units / precision-units (NOT world-units)
	spring_damping: Float,// Force / velocity
	gravity: Float,// world-units/sec^2
	ground_k: Float,
	iterations_per_correction: u64,// Used for conservation of energy correction as well as local relative velocity averaging
	local_relative_velocity_averaging: Float
}

impl Default for SoftBodySettings {
	fn default() -> Self {
		Self {
			precision: 1.0,
			node_mass: 1.0,
			spring_k: 10.0,
			spring_damping: 0.0,
			gravity: -9.81,
			ground_k: 100.0,
			iterations_per_correction: 400,
			local_relative_velocity_averaging: 0.1
		}
	}
}

pub struct SoftBodyAuxData {
	pub energy: Float,
	iterations: u64
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
	pub fn from_rgba_image(grid_size: IntV2, image: &RgbaImage, settings: SoftBodySettings) -> Self {
		let grid_size_float = intv2_to_v2(grid_size);
		Self::from_bb_inclusion_func(
			grid_size,
			|grid_pos| {
				let grid_pos_float = intv2_to_v2(grid_pos);
				let img_pos = ImgV2::new(
					(                  grid_pos_float.x * (image.width()  as Float  / grid_size_float.x)) as u32,
					image.height() - ((grid_pos_float.y * (image.height() as Float) / grid_size_float.y) as u32)
				);
				image.get_pixel(img_pos.x.min(image.width() - 1), img_pos.y.min(image.height() - 1)).0[3] > 128
			},
			settings
		)
	}
	fn apply_iso(&self, state: &mut VDyn, aux_state: &mut SoftBodyAuxData, iso: Iso2) {
		for node_i in 0..self.num_nodes {
			let (pos, vel) = Self::get_node_pos_and_vel_from_state_vec(node_i, state);
			// Rotate pos and vel, only translate pos
			let new_pos = (iso.rotation * pos) + iso.translation.vector;
			let new_vel = iso.rotation * vel;
			// Set new pos and vel
			Self::generic_set_state(node_i, new_pos, new_vel, state);
		}
		// Update energy state
		aux_state.energy = self.total_energy(state);
	}
	/// Returns: (PE, KE)
	pub fn energy(&self, state: &VDyn) -> (Float, Float) {
		let mut pe: Float = 0.0;
		let mut ke: Float = 0.0;
		self.iter_nodes(&mut |grid_pos, node_i| -> () {
			let (pos, vel) = Self::get_node_pos_and_vel_from_state_vec(node_i, state);
			pe += self.settings.node_mass * -self.settings.gravity * pos.y;// PE = mgh
			#[cfg(feature = "spring-energy")] self.iter_adjacent_nodes(&grid_pos, &mut |absolute_grid_pos, grid_offset| -> () {
				// Get "ideal" spring length based on relative grid position
				let grid_offset_float = intv2_to_v2(grid_offset);
				let ideal_length = grid_offset_float.magnitude() * self.settings.precision;
				// Get actual offset
				let (pos1, _) = Self::get_node_pos_and_vel_from_state_vec(node_i, state);
				let (pos2, _) = Self::get_node_pos_and_vel_from_state_vec(self.get_node_i_from_grid_pos(&absolute_grid_pos).expect(&format!("failed to get node index at grid position {:?}", absolute_grid_pos)), state);
				//dbg!(offset);
				// Finally get force
				pe += self.spring_energy((pos2 - pos1).magnitude(), ideal_length) * 0.5;// This will go over each spring twice, so multiply be half to compensate
			});
			ke += self.settings.node_mass * vel.magnitude().powi(2) * 0.5;// KE = mV^2/2
		});
		// Done
		(pe, ke)
	}
	pub fn total_energy(&self, state: &VDyn) -> Float {
		let (pe, ke) = self.energy(state);
		pe + ke
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
	/*fn spring_force_ratio(&self, length: Float) -> Float {
		// Ideal length is 1. + is tension, - is compression
		let length_corrected = length - 1.0;
		length_corrected * self.settings.spring_k
	}*/
	fn spring_force_linear(&self, length: Float, ideal_length: Float) -> Float {
		// Ideal length is 1. + is tension, - is compression
		let length_offset = length - ideal_length;
		length_offset * self.settings.spring_k
	}
	fn spring_force_quadratic(&self, length: Float, ideal_length: Float) -> Float {
		// Ideal length is 1. + is tension, - is compression
		let length_offset = length - ideal_length;
		(length_offset.powi(2) * (sign(length_offset) as Float) + length_offset) * self.settings.spring_k
	}
	fn spring_energy(&self, length: Float, ideal_length: Float) -> Float {
		self.settings.spring_k * 0.5 * (length - ideal_length).powi(2)
	}
	fn get_node_i_from_grid_pos(&self, grid_pos: &IntV2) -> Option<usize> {
		match self.node_index_lookup.get(grid_pos) {
			Some(i) => Some(*i),
			None => None
		}
	}
	fn node_net_force(&self, node_i: usize, node_grid_pos: IntV2, state: &VDyn) -> V2 {
		let mut net_force = V2::zeros();
		assert_eq!(self.get_node_i_from_grid_pos(&node_grid_pos).expect("Unable to get node index during assertion"), node_i);
		self.iter_adjacent_nodes(
			&node_grid_pos,
			&mut |absolute_grid_pos, grid_offset| -> () {
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
		);
		// Done
		net_force
	}
	/// Iterates over nodes adjacent to `center_node_grid_pos` with `func`. `func` must have params (absolute grid pos of adjacent node, grid offset to adjacent node)
	fn iter_adjacent_nodes(&self, center_node_grid_pos: &IntV2, func: &mut impl FnMut(IntV2, IntV2) -> ()) {
		for x_offset in -1..2 {
			for y_offset in -1..2 {
				let grid_offset = IntV2::new(x_offset, y_offset);
				let absolute_grid_pos = center_node_grid_pos + grid_offset;
				if grid_offset != IntV2::zeros() && absolute_grid_pos.x >= 0 && absolute_grid_pos.x < self.bounding_box.x && absolute_grid_pos.y >= 0 && absolute_grid_pos.y < self.bounding_box.y {
					if self.does_node_exist_at_grid_pos(&absolute_grid_pos) {
						func(absolute_grid_pos, grid_offset);
					}
				}
			}
		}
	}
	fn local_relative_velocity_averaging(&self, state: &mut VDyn, aux_state: &mut SoftBodyAuxData) {
		// Get local averages
		let mut relative_velocity_averages = HashMap::<usize, V2>::new();
		self.iter_nodes(&mut |grid_pos, node_i| {
			let mut v_sum = Self::get_node_pos_and_vel_from_state_vec(node_i, state).1;
			let mut count: usize = 1;
			self.iter_adjacent_nodes(&grid_pos, &mut |adj_grid_pos, _| {
				let (_, adj_vel) = Self::get_node_pos_and_vel_from_state_vec(self.get_node_i_from_grid_pos(&adj_grid_pos).expect("Could not get node index from grid pos when iterating adjacent nodes"), state);
				v_sum += adj_vel;
				count += 1;
			});
			// Calculate average velocity of this node along with its neighbors
			let avg_vel = v_sum / (count as Float);
			relative_velocity_averages.insert(node_i, avg_vel);
		});
		// Lerp towards average
		self.iter_nodes(&mut |grid_pos, node_i| {
			let (pos, curr_vel) = Self::get_node_pos_and_vel_from_state_vec(node_i, state);
			let local_avg_vel = relative_velocity_averages.get(&node_i).expect("Unable to get local relative velocity from hashmap which should have every node");
			let new_vel = curr_vel.lerp(&local_avg_vel, self.settings.local_relative_velocity_averaging);
			Self::generic_set_state(node_i, pos, new_vel, state);
		});
		// Update enrgy state
		aux_state.energy = self.total_energy(state);
	}
	fn force_between_nodes(&self, ideal_length: Float, rel_pos: V2, rel_vel: V2) -> V2 {
		(rel_pos / rel_pos.magnitude()) * (
			self.spring_force_quadratic(rel_pos.magnitude(), ideal_length)// Static spring force
			+ self.spring_damping_force(rel_pos, rel_vel)// Damping, project rel_vel onto rel_pos and multiply magitude by self.settings.spring_damping
		)
	}
	pub fn spring_damping_force(&self, rel_pos: V2, rel_vel: V2) -> Float {
		//v2_project(rel_vel, rel_pos).magnitude() * self.settings.spring_damping
		if rel_vel.magnitude() <= EPSILON || rel_pos.magnitude() <= EPSILON {
			return 0.0;
		}
		let onto_normalized = rel_pos / rel_pos.magnitude();
		let dot_prod = v2_dot(rel_vel, onto_normalized);
        match dot_prod.is_finite() {
			true => dot_prod,
			false => 0.0
		}
	}
	fn does_node_exist_at_grid_pos(&self, grid_pos: &IntV2) -> bool {
		self.fill[grid_pos.x as usize][grid_pos.y as usize]
	}
	pub fn render_image(&self, state: &VDyn, image: &mut RgbImage, #[cfg(feature = "texture-rendering")] texture: &RgbaImage, fill_color: Rgb<u8>, translater: &ImagePosTranslater) {
		self.iter_nodes(&mut |grid_pos, node_i| -> () {
			#[cfg(feature = "node-point-rendering")] {
				let (pos, _) = Self::get_node_pos_and_vel_from_state_vec(node_i, state);
				match translater.world_to_px(pos) {
					Some(px_pos) => image.put_pixel(px_pos.x, px_pos.y, fill_color),
					None => {}
				}
			}
			#[cfg(feature = "texture-rendering")] {
				
			}
		});
		// Ground
		let ground_px_y = translater.world_to_px(V2::zeros()).expect("Expected world origin to be on the image, this is not the greatest code though so it could get broken").y;
		for px_x in 0..image.width() {
			image.put_pixel(px_x, ground_px_y, Rgb([255; 3]));
		}
	}
	/// Iterate over each node in the grid and run `func`. `func` should take (grid pos, node index)
	fn iter_nodes(&self, func: &mut impl FnMut(IntV2, usize) -> ()) {
		let mut node_i: usize = 0;
		for x in 0..self.bounding_box.x {
			for y in 0..self.bounding_box.y {
				let grid_pos = IntV2::new(x, y);
				let is_node: bool = self.does_node_exist_at_grid_pos(&grid_pos);
				if is_node {
					func(grid_pos, node_i);
					node_i += 1;
				}
			}
		}
		// Safety check
		assert_eq!(node_i, self.num_nodes, "When iterating over all nodes, the number of occupied nodes counted ({}) was != to `self.num_nodes` ({})", node_i, self.num_nodes);
	}
}

impl StaticDifferentiator for SoftBody {
	/* The vector state representation will have the following format for each node index `i`
	i*4 + 0: X position
	i*4 + 1: Y position
	i*4 + 2: X velocity
	i*4 + 3: Y velocity
	*/
	type AuxData = SoftBodyAuxData;
	fn state_representation_vec_size(&self) -> usize {
		// (Number of nodes * 2 dimensions) * 2 (positions and velocities)
		self.num_nodes * 4
	}
	fn initial_state(&self) -> (VDyn, SoftBodyAuxData) {
		let mut out = VDyn::from_vec(vec![0.0; self.state_representation_vec_size()]);
		self.iter_nodes(&mut |grid_pos, node_i| -> () {
			// Set node position and velocity
			let start = node_i * 4;
			// Position
			out[start    ] = (grid_pos.x as Float) * self.settings.precision;
			out[start + 1] = (grid_pos.y as Float) * self.settings.precision;
			// Velocity
			out[start + 2] = 0.0;
			out[start + 3] = 0.0;
		});
		// Done
		let energy = self.total_energy(&out);
		(out, SoftBodyAuxData{energy, iterations: 0})
	}
	fn differentiate(&self, state: &VDyn, aux_state: &mut SoftBodyAuxData) -> NDimensionalDerivative {
		let mut derivative = NDimensionalDerivative(VDyn::from_vec(vec![0.0; self.state_representation_vec_size()]));
		self.iter_nodes(&mut |grid_pos, node_i| -> () {
			// Calculate acceleration
			let force: V2 = self.node_net_force(node_i, grid_pos, state);
			let acc: V2 = (force / self.settings.node_mass) + V2::new(0.0, self.settings.gravity);
			let (_, vel) = Self::get_node_pos_and_vel_from_state_vec(node_i, state);
			Self::set_node_vel_and_acc(node_i, vel, acc, &mut derivative);
		});
		assert_vec_is_finite(&derivative.0).unwrap();
		aux_state.iterations += 1;
		// Done
		derivative
	}
	fn set_state(&self, state: &mut VDyn, aux_state: &mut Self::AuxData) {
		if aux_state.iterations % self.settings.iterations_per_correction == 0 {
			#[cfg(feature = "conservation-of-energy")] {
				// Enforce conservation of energy. It is the law.
				assert!(aux_state.energy >= 0.0, "Energy must not be >= 0, otherwise this will cause problems with the energy conservation correction");
				let ideal_total = aux_state.energy;
				let (actual_pe, actual_ke) = self.energy(state);
				let ideal_ke = ideal_total - actual_pe;
				let energy_ratio = actual_ke / ideal_ke;
				// Scale velocity for each node to adjust actual KE
				for node_i in 0..self.num_nodes {
					let (pos, vel) = Self::get_node_pos_and_vel_from_state_vec(node_i, state);
					let new_vel = vel / energy_ratio.sqrt();// Pretty sure this is correct, TODO: verify
					Self::generic_set_state(node_i, pos, new_vel, state);
				}
				// Check
				assert_relative_eq!(self.total_energy(state), aux_state.energy, epsilon = EPSILON);
			}
			#[cfg(feature = "local-relative-velocity-averaging")] {
				self.local_relative_velocity_averaging(state, aux_state);
			}
		}
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
	// Texture image
	#[cfg(feature = "texture-rendering")]
	let texture = {// Current texture is 176 x 493, scaling for height = 100, width = 36
		let texture_dyn = Reader::open(format!("{}texture.png", MEDIA_DIR)).unwrap().decode().unwrap();
		texture_dyn.into_rgba8()
	};
	let body = SoftBody::from_bb_inclusion_func(
		IntV2::new(40, 40),// 30 x 30
		|grid_pos: IntV2| {
			let grid_pos_float = intv2_to_v2(grid_pos);
			let center = V2::new(20.0, 20.0);
			(grid_pos_float - center).magnitude() <= 20.0
		},
		SoftBodySettings {
			precision: 1.0,
			node_mass: 1.0,// 0.7
			spring_k: 1000.0,// 2000.0
			spring_damping: 0.17,
			gravity: -1.0,// -1.0
			ground_k: 300.0,
			iterations_per_correction: 800,// 800
			local_relative_velocity_averaging: 0.0// Don't use
		}
	);
	let num_frames: usize = 400;
	let iterations_per_frame: usize = 125;// 250
	let dt = 0.0016;// 0.0008
	let background_color = Rgb([0; 3]);
	let fill_color = Rgb([255; 3]);
	let image_size = ImgV2::new(750, 750);
	let translater = ImagePosTranslater {
		scale: 12.0,// 15.0
		origin: V2::new(8.0, 25.0),// 8.0, 20.0
		image_size
	};
	let mut stepper = Stepper::new(body, 10000.0, dt);
	// Transform body
	//SoftBody::generic_set_state(0, V2::new(-5.0, 0.0), V2::zeros(), &mut stepper.state);// Give nodes opposing velocities from spring damping testing
	//SoftBody::generic_set_state(1, V2::new(5.0, 0.0), V2::zeros(), &mut stepper.state);
	stepper.differentiator.apply_iso(&mut stepper.state, &mut stepper.aux_state, Iso2{rotation: UnitComplex::from_angle(PI/6.0), translation: Translation{vector: V2::new(0.0, 5.0)}});// PI/6.0, 0.0, 5.0
	// Build video creator
	let mut video_creator = VideoCreator {
		num_frames,
		iterations_per_frame,
		background_color,
		fill_color,
		image_size,
		translater,
		stepper,
		#[cfg(feature = "texture-rendering")]
		texture
	};
	video_creator.create();
}

pub struct VideoCreator {
	num_frames: usize,
	iterations_per_frame: usize,
	background_color: Rgb<u8>,
	fill_color: Rgb<u8>,
	image_size: ImgV2,
	translater: ImagePosTranslater,
	stepper: Stepper<SoftBody>,
	#[cfg(feature = "texture-rendering")]
	texture: RgbaImage
}

impl VideoCreator {
	pub fn create(&mut self) {
		for i in 0..self.num_frames {
			// Step
			for _ in 0..self.iterations_per_frame {
				self.stepper.step();
			}
			// Render image
			let mut image: RgbImage = ImageBuffer::from_pixel(self.image_size.x, self.image_size.y, self.background_color);
			self.stepper.differentiator.render_image(&self.stepper.state, &mut image, #[cfg(feature = "texture-rendering")] &self.texture, self.fill_color, &self.translater);
			// Save image
			image.save(&format!("{}soft_body_render_{}.png", MEDIA_DIR, i));
		}
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
			ground_k: 100.0,
			iterations_per_correction: 400,
			local_relative_velocity_averaging: 0.1
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
		let (mut state, _) = body.initial_state();
		assert_eq!(body.state_representation_vec_size(), 8);
		assert_eq!(SoftBody::get_node_pos_and_vel_from_state_vec(0, &state), (V2::zeros(), V2::zeros()));
		assert_eq!(SoftBody::get_node_pos_and_vel_from_state_vec(1, &state), (V2::new(1.0, 0.0), V2::zeros()));
		// ----------------------- Spring force (-kx) -----------------------
		// 1
		// Change 2nd node to be 2 units away from the first one
		SoftBody::generic_set_state(1, V2::new(2.0, 0.0), V2::zeros(), &mut state);
		// Calculate force
		assert_eq!(body.node_net_force(1, IntV2::new(1, 0), &state), V2::new(-30.0, 0.0));
		// 2
		// Change 2nd node to be 0.5 units away from the first one
		SoftBody::generic_set_state(1, V2::new(0.5, 0.0), V2::zeros(), &mut state);
		// Calculate force
		assert_eq!(body.node_net_force(1, IntV2::new(1, 0), &state), V2::new(15.0 * 0.75, 0.0));
		// ----------------------- Spring damping -----------------------
		// 3
		// Change 2nd node to be at preferable distance (precision = 1 unit) away from 1st one, but it has a velocity going away from it
		SoftBody::generic_set_state(1, V2::new(1.0, 0.0), V2::new(3.0, 1000.0), &mut state);// that big Y velocity shouldn't affect anything
		// Calculate force
		assert_eq!(body.node_net_force(1, IntV2::new(1, 0), &state), V2::new(-3.0, 0.0));
		// 4
		// Opposite velocity from previous test
		SoftBody::generic_set_state(1, V2::new(1.0, 0.0), V2::new(-3.0, 1000.0), &mut state);// that big Y velocity shouldn't affect anything
		// Calculate force
		assert_eq!(body.node_net_force(1, IntV2::new(1, 0), &state), V2::new(3.0, 0.0));
	}
	#[test]
	#[cfg(feature = "conservation-of-energy")]
	fn energy() {
		let settings = soft_body_settings();
		let body = SoftBody::from_bb_inclusion_func(IntV2::new(1, 2), |_| {true}, settings.clone());
		let (mut state, mut aux_state) = body.initial_state();
		assert_eq!(body.state_representation_vec_size(), 8);// 2 nodes x 4 scalars per node
		// 1
		// Node 0 will have no energy, node 1 will have gravity * mass * Y-value potential energy
		assert_eq!(aux_state.energy, -settings.gravity);
		assert_eq!(body.energy(&state), (-settings.gravity, 0.0));
		// 2
		// Rotate 90 degrees so both nodes are @ y=0
		body.apply_iso(&mut state, &mut aux_state, Iso2{rotation: UnitComplex::from_angle(PI/2.0), translation: Translation{vector: V2::zeros()}});
		assert_relative_eq!(aux_state.energy, 0.0, epsilon = EPSILON);
		// 3
		// Set velocity and position
		SoftBody::generic_set_state(1, V2::new(-1.0, 1.0), V2::new(0.0, 5.0), &mut state);
		assert_eq!(body.energy(&mut state), (-settings.gravity, 12.5));
	}
	#[cfg(feature = "conservation-of-energy")]
	#[test]
	fn energy_conservation_correction() {
		let settings = soft_body_settings();
		let body = SoftBody::from_bb_inclusion_func(IntV2::new(1, 2), |_| {true}, settings.clone());
		let (mut state, mut aux_state) = body.initial_state();
		assert_eq!(aux_state.energy, -settings.gravity);
		// 1
		// Move node 1 to lower y-value and increase its velocity by a lot
		SoftBody::generic_set_state(1, V2::new(0.0, 0.5), V2::new(10.0, 0.0), &mut state);
		assert_eq!(body.energy(&state), (-settings.gravity / 2.0, 50.0));
		body.set_state(&mut state, &mut aux_state);
		assert_relative_eq!(body.total_energy(&state), aux_state.energy);
	}
}