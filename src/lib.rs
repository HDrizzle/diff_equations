/* Differential equations viewer and simple circuit simulator, created 2024-1-8 by Hadrian Ward for MSSM J-Term
Inspired by: Computers pattern chaos and beauty, by Clifford A Pickover. Pg 216
*/
#![allow(warnings)]// Trust me bro

use std::{ops, f32::consts::PI};
use bevy::ecs::system::Resource;
use nalgebra::{Scalar, Dim, Const, Matrix, VecStorage, base::{Vector2, Vector3, OVector, dimension::Dyn}, Isometry2};
use approx::assert_relative_eq;
use geo::geometry::Coord;

//pub mod electronics;
//pub mod gui;
pub mod physics;

pub mod prelude {
	use super::*;
	pub const APP_NAME: &str = "Differential equation plotter";
	pub const MEDIA_DIR: &str = "media/";
	pub type Float = f64;
	pub const EPSILON: Float = 10e-6;
	pub type Int = i32;
	pub type UInt = u32;
	pub type V2 = Vector2<Float>;
	pub type V3 = Vector3<Float>;
	pub type VDyn = OVector<Float, Dyn>;
	pub type ImgV2 = Vector2<u32>;
	pub type IntV2 = Vector2<Int>;
	pub type Iso2 = Isometry2<Float>;
	pub use std::f64::consts::PI;
	use geo::Coord;
pub use image::{RgbImage, ImageBuffer, Rgb, io::Reader, DynamicImage};
	/*pub fn is_point_inside_triangle(p: IntV2, a: IntV2, b: IntV2, c: IntV2) -> bool {
		is_point_inside_triangle_counter_clockwise(p, a, b, c) || is_point_inside_triangle_counter_clockwise(p, b, a, c)// one is reversed, so it should work both ways
	}
	pub fn is_point_inside_triangle_counter_clockwise(p: IntV2, a: IntV2, b: IntV2, c: IntV2) -> bool {// From ChatGPT
		// Calculate barycentric coordinates
		let v0 = b - a;
		let v1 = c - a;
		let v2 = p - a;
	
		let dot00 = v0.x * v0.x + v0.y * v0.y;
		let dot01 = v0.x * v1.x + v0.y * v1.y;
		let dot02 = v0.x * v2.x + v0.y * v2.y;
		let dot11 = v1.x * v1.x + v1.y * v1.y;
		let dot12 = v1.x * v2.x + v1.y * v2.y;
	
		// Compute barycentric coordinates
		let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01) as Float;
		let u = (dot11 * dot02 - dot01 * dot12) as Float * inv_denom;
		let v = (dot00 * dot12 - dot01 * dot02) as Float * inv_denom;
	
		// Check if the point is inside the triangle
		(u >= 0.0) && (v >= 0.0) && (u + v <= 1.0)
	}*/
	pub fn v2_to_geo_coord(v: V2) -> Coord<Float> {
		Coord{x: v.x, y: v.y}
	}
	pub fn sign(n: Float) -> Int {
		match n >= 0.0 {
			true => 1,
			false => -1
		}
	}
	pub fn imgv2_to_v2(imgv2: ImgV2) -> V2 {
		V2::new(
			imgv2.x.into(),
			imgv2.y.into()
		)
	}
	pub fn v2_to_imgv2(v2: V2) -> ImgV2 {
		ImgV2::new(
			v2.x as u32,
			v2.y as u32
		)
	}
	pub fn v2_to_intv2(v2: V2) -> IntV2 {
		IntV2::new(
			v2.x as Int,
			v2.y as Int
		)
	}
	pub fn intv2_to_v2(v2: IntV2) -> V2 {
		V2::new(
			v2.x as Float,
			v2.y as Float
		)
	}
	pub fn intv2_to_imgv2(v2: IntV2) -> ImgV2 {
		ImgV2::new(
			v2.x as u32,
			v2.y as u32
		)
	}
	pub fn imgv2_to_intv2(imgv2: ImgV2) -> IntV2 {
		IntV2::new(
			imgv2.x as Int,
			imgv2.y as Int
		)
	}
	pub fn v2_dot(v1: V2, v2: V2) -> Float {
		v1.x * v2.x + v1.y * v2.y
	}
	pub fn v2_project(v: V2, onto: V2) -> V2 {// https://www.chasing-carrots.com/the-pragmatical-programmers-guide-to-vectors/
		let onto_normalized = onto / onto.magnitude();
		let dot_prod = v2_dot(v, onto_normalized);

        onto_normalized * dot_prod
	}
	pub use crate::{
		NDimensionalDerivative,
		StaticDifferentiator,
		PlotVariableIndices,
		Stepper,
		ImagePosTranslater,
		physics::{
			StaticSpringAndMass,
			soft_body
		},
		render_image,
		assert_vec_is_finite
	};
}

use prelude::*;

#[derive(Clone, PartialEq, Debug)]
pub struct NDimensionalDerivative (
	pub VDyn
);

pub struct PlotVariableIndices (// Array of indices corresponding to `GeneralNumericState` array
	pub Vec<usize>
);

impl PlotVariableIndices {
	pub fn get_plot_variables(&self, state: &VDyn) -> Vec<Float> {
		let mut out = Vec::<Float>::new();
		for i in &self.0 {
			out.push(state[*i]);
		}
		// Done
		out
	}
}

pub trait StaticDifferentiator {
	type AuxData;
	fn state_representation_vec_size(&self) -> usize;
	fn initial_state(&self) -> (VDyn, Self::AuxData);
	fn differentiate(&self, state: &VDyn, aux_state: &mut Self::AuxData) -> NDimensionalDerivative;
	fn set_state(&self, state: &mut VDyn, aux_state: &mut Self::AuxData) {}
}

#[derive(Resource)]
pub struct Stepper<T: StaticDifferentiator> {
	pub differentiator: T,
	state_vec_len_limit: Float,
	pub state: VDyn,
	pub aux_state: T::AuxData,
	dt: Float,
	iterations: u64
}

impl<T: StaticDifferentiator> Stepper<T> {
	pub fn new(differentiator: T, state_vec_len_limit: Float, dt: Float) -> Self {
		let (state, aux_state) = differentiator.initial_state();
		assert_vec_is_finite(&state).unwrap();
		Self {
			differentiator,
			state_vec_len_limit,
			state,
			aux_state,
			dt,
			iterations: 0
		}
	}
	pub fn step(&mut self) -> Result<(), String> {
		let final_state = {
			#[cfg(not(feature = "single-step"))] {
				// Steps state by 2/3 twice and averages result after each step, I figured this out a while ago and it is very stable and should prevent oscillation
				// Step 1
				let mut diff1: NDimensionalDerivative = self.differentiator.differentiate(&self.state, &mut self.aux_state);
				let mut state1: VDyn = self.state.clone() + (diff1.0 * self.dt * (2.0 / 3.0));
				self.differentiator.set_state(&mut state1, &mut self.aux_state);
				// Step 2
				let diff2: NDimensionalDerivative = self.differentiator.differentiate(&state1, &mut self.aux_state);
				let mut state2: VDyn = state1.clone() + (diff2.0 * self.dt * (2.0 / 3.0));
				self.differentiator.set_state(&mut state2, &mut self.aux_state);
				// Average state
				let mut final_state = (state1 + state2) / 2.0;
				final_state
			}
			#[cfg(feature = "single-step")] {
				let mut diff: NDimensionalDerivative = self.differentiator.differentiate(&self.state, &mut self.aux_state);
				let mut new_state: VDyn = self.state.clone() + (diff.0 * self.dt);
				self.differentiator.set_state(&mut new_state, &mut self.aux_state);
				new_state
			}
		};
		// Done
		if final_state.magnitude() >= self.state_vec_len_limit {// I'm pretty sure magnitude works in this context, but not certain
			return Err(format!("Vector magnitude of state met or exceeded limit of {}", self.state_vec_len_limit));
		}
		assert_vec_is_finite(&final_state).unwrap();
		self.iterations += 1;
		self.state = final_state;
		Ok(())
	}
}

pub struct ImagePosTranslater {
	pub scale: Float,// Px per world units, gets larger when zoomed in
	pub origin: V2,// Location of world origin relative to center of image image (px), not effected by scale
	pub image_size: ImgV2
}

impl ImagePosTranslater {
	pub fn world_to_px(
		&self,
		input: V2,
	) -> (IntV2, bool) {
		// Returns (Image pos, whether pos is on image)
		let center = self.image_size / 2;
		let final_y_up = v2_to_intv2((input - self.origin) * self.scale) + imgv2_to_intv2(center);
		let px_pos = IntV2::new(final_y_up.x, self.image_size.y as Int - final_y_up.y);
		let (x, y) = (px_pos.x, px_pos.y);
		(
			px_pos,
			x < self.image_size.x as Int && x >= 0 && y < self.image_size.y as Int && y >= 0
		)
	}
	pub fn px_to_world(
		&self,
		px_int: IntV2,
	) -> V2 {
		let px_y_down = intv2_to_v2(px_int);
		let px = V2::new(px_y_down.x, self.image_size.y as Float - px_y_down.y);
		let center = imgv2_to_v2(self.image_size / 2);
		((px - center) / self.scale) + self.origin
	}
}

pub fn render_image<T: StaticDifferentiator>(
	mut stepper: Stepper<T>,
	plot_indices: &PlotVariableIndices,
	image: &mut RgbImage,
	translater: ImagePosTranslater,// Location of world origin relative to center of image image (px), not effected by scale
	num_iterations: UInt,
	fill_color: Rgb<u8>
) {
	assert_eq!(plot_indices.0.len(), 2, "Plot indices must have length of 2 for image rendering");
	for _ in 0..num_iterations {
		stepper.step();
		let pos = plot_indices.get_plot_variables(&stepper.state);
		let (px_pos, is_on_image) = translater.world_to_px(V2::new(pos[0], pos[1]));
		if is_on_image {
			image.put_pixel(px_pos.x as u32, px_pos.y as u32, fill_color);
		}
		/*let px_pos = to_px(V2::new(pos[0], pos[1]));
		let (x, y) = (px_pos.x, px_pos.y);
		if x < image.width() as Int && x >= 0 && y < image.height() as Int && y >= 0 {
			image.put_pixel(px_pos.x as u32, px_pos.y as u32, fill_color);
		}*/
	}
}

pub fn assert_vec_is_finite(v: &VDyn) -> Result<(), String> {
	for (i, n) in v.iter().enumerate() {
		if !n.is_finite() {
			return Err(format!("Scalar at index {} is not finite ({})", i, n));
		}
	}
	Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
	#[test]
	fn v2_projection() {
		assert_relative_eq!(v2_project(V2::new(1.0, -2.0), V2::new(1.0, 0.0)), V2::new(1.0, 0.0), epsilon = EPSILON);
		assert_relative_eq!(v2_project(V2::new(0.0, -10.0), V2::new(-20.0, -20.0)), V2::new(-5.0, -5.0), epsilon = EPSILON);
		assert_relative_eq!(v2_project(V2::new(-1.0, 4.0), V2::new(4.0, 1.0)), V2::new(0.0, 0.0), epsilon = EPSILON);
	}
	mod image_translation {
		use super::*;
		fn new_translater() -> ImagePosTranslater {
			ImagePosTranslater{
				scale: 150.0,
				origin: V2::new(10.0, 20.0),
				image_size: ImgV2::new(500, 500)
			}
		}
		#[test]
		fn composition() {
			let translater = new_translater();
			assert_eq!(translater.px_to_world(translater.world_to_px(V2::zeros()).0), V2::zeros());
			assert_eq!(translater.world_to_px(translater.px_to_world(IntV2::new(30, 40))).0, IntV2::new(30, 41));// fine
		}
	}
}