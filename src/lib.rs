/* Differential equations viewer and simple circuit simulator, created 2024-1-8 by Hadrian Ward for MSSM J-Term
Inspired by: Computers pattern chaos and beauty, by Clifford A Pickover. Pg 216
*/
#![allow(warnings)]// Trust me bro

use std::ops;
use bevy::ecs::system::Resource;
use image::{RgbImage, Rgb};
use nalgebra::{Scalar, base::{Vector2, Vector3}};

pub mod electronics;
pub mod spring;
pub mod gui;

pub mod prelude {
	use super::*;
	pub const APP_NAME: &str = "Differential equation plotter";
	pub type Float = f64;
	pub type Int = i32;
	pub type UInt = u32;
	pub type V2 = Vector2<Float>;
	pub type V3 = Vector3<Float>;
	pub type ImgV2 = Vector2<u32>;
	pub type IntV2 = Vector2<Int>;
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
	pub fn imgv2_to_intv2(imgv2: ImgV2) -> IntV2 {
		IntV2::new(
			imgv2.x as Int,
			imgv2.y as Int
		)
	}
	pub use crate::{
		GenericVector,
		NDimensionalDerivative,
		StaticDifferentiator,
		PlotVariableIndices,
		Stepper,
		spring,
		render_image
	};
}

use prelude::*;

#[derive(Clone, PartialEq, Debug)]
pub struct GenericVector<const N: usize> (
	pub [Float; N]
);

impl<const N: usize> GenericVector<N> {
	pub fn average(&mut self, other: Self) {
		let mut out = [0.0; N];
		for (i, n0) in self.0.iter().enumerate() {
			out[i] = (n0 + other.0[i]);
		}
		self.0 = out;
	}
	pub fn len(&self) -> Float {
		let mut sum = 0.0;
		for n in &self.0 {
			sum += n.powf(2.0);
		}
		sum.powf(0.5)
	}
	pub fn new() -> Self {
		Self([0.0; N])
	}
	pub fn mul_by_scalar(&self, scalar: Float) -> Self {
		let mut out = [0.0; N];
		for (i, n0) in self.0.iter().enumerate() {
			out[i] = self.0[i] * scalar;
		}
		Self (
			out
		)
	}
	pub fn add(&self, other: Self) -> Self {
		let mut out = [0.0; N];
		for (i, n0) in self.0.iter().enumerate() {
			out[i] = self.0[i] + other.0[i];
		}
		Self (
			out
		)
	}
	pub fn get_plot_variables(&self, plot_variable_indices: &PlotVariableIndices) -> Vec<Float> {
		let mut out = Vec::<Float>::new();
		for i in &plot_variable_indices.0 {
			assert!(i < &N, "Index ({}) from plot variable indices must be < length of this generic vector ({})", i, N);
			out.push(self.0[*i]);
		}
		// Done
		out
	}
}

#[derive(Clone, PartialEq, Debug)]
pub struct NDimensionalDerivative<const N: usize> (
	pub GenericVector<N>
);

pub struct PlotVariableIndices(// Array of indices corresponding to `GeneralNumericState` array
	pub Vec<usize>
);

pub trait StaticDifferentiator<const N: usize>: Clone + Send {// N: size of display variable array, M: size of general numeric state
	fn new() -> Self;
	fn differentiate(&mut self, state: &GenericVector<N>) -> NDimensionalDerivative<N>;
}

#[derive(Resource)]
pub struct Stepper<const N: usize, T: StaticDifferentiator<N>> {
	differentiator: T,
	state_vec_len_limit: Float,
	pub state: GenericVector<N>,
	dt: Float
}

impl<const N: usize, T: StaticDifferentiator<N>> Stepper<N, T> {
	pub fn new(state_vec_len_limit: Float, dt: Float) -> Self {
		Self {
			differentiator: T::new(),
			state_vec_len_limit,
			state: GenericVector::<N>::new(),
			dt
		}
	}
	pub fn step(&mut self) -> Result<(), String> {
		let final_state = /*{
			// Steps state by 2/3 twice and averages result after each step, I figured this out a while ago and it is very stable and prevents oscillation
			// Step 1
			let mut diff1: NDimensionalDerivative<N> = self.differentiator.differentiate(&self.state);
			let state1: GenericVector<N> = self.state.add(diff1.0.mul_by_scalar(self.dt * (2.0 / 3.0)));
			// Step 2
			let diff2: NDimensionalDerivative<N> = self.differentiator.differentiate(&state1);
			let state2: GenericVector<N> = state1.add(diff2.0.mul_by_scalar(self.dt * (2.0 / 3.0)));
			// Average state
			let mut final_state = state1;
			final_state.average(state2);
			final_state
		}*/
		{
			let mut diff: NDimensionalDerivative<N> = self.differentiator.differentiate(&self.state);
			let new_state: GenericVector<N> = self.state.add(diff.0.mul_by_scalar(self.dt));
			new_state
		};
		// Done
		if final_state.len() >= self.state_vec_len_limit {
			return Err(format!("Vector magnitude of state met or exceeded limit of {}", self.state_vec_len_limit));
		}
		self.state = final_state;
		Ok(())
	}
}

pub fn render_image<const N: usize, T: StaticDifferentiator<N>>(
	mut stepper: Stepper<N, T>,
	plot_indices: &PlotVariableIndices,
	image: &mut RgbImage,
	scale: Float,// Px per world units, gets larger when zoomed in
	origin: ImgV2,// Location of world origin relative to center of image image (px), not effected by scale
	num_iterations: UInt,
	fill_color: Rgb<u8>
) {
	assert_eq!(plot_indices.0.len(), 2, "Plot indices must have length of 2 for image rendering");
	let size = ImgV2::new(image.width(), image.height());
	let center = size / 2;
	// Temp function to translate into pixel units
	let to_px = |input: V2| -> IntV2 {
		let final_y_up = v2_to_intv2(input * scale) + imgv2_to_intv2(center);// TODO: apply origin
		IntV2::new(final_y_up.x, size.y as Int - final_y_up.y)
	};
	for _ in 0..num_iterations {
		stepper.step();
		let pos = stepper.state.get_plot_variables(plot_indices);
		let px_pos = to_px(V2::new(pos[0], pos[1]));
		let (x, y) = (px_pos.x, px_pos.y);
		if x < image.width() as Int && x >= 0 && y < image.height() as Int && y >= 0 {
			image.put_pixel(px_pos.x as u32, px_pos.y as u32, fill_color);
		}
	}
}