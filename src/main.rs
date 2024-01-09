// Most functionality is in lib.rs, this is only for playing around
#![allow(warnings)]
use diff_equations::prelude::*;
use image::{Rgb, ImageBuffer, RgbImage};
use nalgebra::max;

fn create_image() {
	let background = Rgb([0; 3]);
	let fill_color = Rgb([255; 3]);
	let num_iterations = 2000;
	let max_magnitude = 10000.0;
	let step = 0.005;
	let scale = 200.0;
	let origin = ImgV2::zeros();
	let mut stepper: Stepper<2, StaticSpringAndMass> = Stepper::<2, StaticSpringAndMass>::new(StaticSpringAndMass::default(), max_magnitude, step);
	stepper.state = GenericVector([0.0, 1.0]);
	let mut image: RgbImage = ImageBuffer::from_pixel(500, 500, background);
	// Render
	render_image(
		stepper,
		&PlotVariableIndices(vec![0,1]),
		&mut image,
		scale,
		origin,
		num_iterations,
		fill_color
	);
	// Save
	image.save("phase_portrait.png");
}

fn start_gui() {
	let stepper: Stepper<2, StaticSpringAndMass> = Stepper::<2, StaticSpringAndMass>::new(StaticSpringAndMass::default(), 10000.0, 0.01);
	diff_equations::gui::main(stepper);
}

fn debug_print() {
	let num_iterations = 10;
	let step = 0.05;
	let max_magnitude = 10000.0;
	let mut stepper: Stepper<2, StaticSpringAndMass> = Stepper::<2, StaticSpringAndMass>::new(StaticSpringAndMass::default(), max_magnitude, step);
	stepper.state = GenericVector([0.0, 1.0]);
	for i in 0..num_iterations {
		println!("{}: {:?}", i, stepper.state);
		stepper.step();
	}
}

fn main() {
	create_image();
}