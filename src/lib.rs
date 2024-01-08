/* Differential equations viewer and simple circuit simulator, created 2024-1-8 by Hadrian Ward for MSSM J-Term
Inspired by: Computers pattern chaos and beauty, by Clifford A Pickover. Pg 216
*/
#![allow(warnings)]// Trust me bro

use std::ops;

pub mod electronics;
pub mod spring;
pub mod gui;

pub mod prelude {
	use super::*;
	pub const APP_NAME: &str = "Differential equation plotter";
	pub use crate::{
		GenericVector,
		NDimensionalDerivative,
		StaticDifferentiator,
		PlotVariables,
		Stepper,
		spring
	};
}

#[derive(Clone)]
pub struct GenericVector<const N: usize> (
	pub [f64; N]
);

impl<const N: usize> GenericVector<N> {
	pub fn average(&mut self, other: Self) {
		let mut out = [0.0; N];
		for (i, n0) in self.0.iter().enumerate() {
			out[i] = (n0 + other.0[i]);
		}
		self.0 = out;
	}
	pub fn new() -> Self {
		Self([0.0; N])
	}
	pub fn mul_by_scalar(&self, scalar: f64) -> Self {
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
}

#[derive(Clone)]
pub struct NDimensionalDerivative<const N: usize> (
	pub GenericVector<N>
);

pub struct PlotVariables(// Array of indices corresponding to `GeneralNumericState` array
	pub Vec<usize>
);

pub trait StaticDifferentiator<const N: usize> {// N: size of display variable array, M: size of general numeric state
	fn new() -> Self;
	fn differentiate(&mut self, state: &GenericVector<N>) -> NDimensionalDerivative<N>;
}

pub struct Stepper<const N: usize, T: StaticDifferentiator<N> + Clone> {
	differentiator: T,
	state: GenericVector<N>,
	dt: f64
}

impl<const N: usize, T: StaticDifferentiator<N> + Clone> Stepper<N, T> {
	pub fn new(dt: f64) -> Self {
		Self {
			differentiator: T::new(),
			state: GenericVector::<N>::new(),
			dt
		}
	}
	pub fn step(&mut self) {
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
		// Done
		self.state = final_state;
	}
}