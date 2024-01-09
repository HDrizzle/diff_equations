// Most functionality is in lib.rs, this is only for playing around
#![allow(warnings)]
use diff_equations::prelude::*;
use image::{Rgb, ImageBuffer, RgbImage};
use nalgebra::max;

fn create_image() {
	let background = Rgb([0; 3]);
	let fill_color = Rgb([255; 3]);
	let num_iterations = 50;
	let max_magnitude = 10000.0;
	let step = 0.1;
	let scale = 1.0;
	let origin = ImgV2::zeros();
	let mut stepper: Stepper<2, spring::StaticSpringAndMass> = Stepper::<2, spring::StaticSpringAndMass>::new(max_magnitude, step);
	stepper.state = GenericVector([200.0, 200.0]);
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
	let stepper: Stepper<2, spring::StaticSpringAndMass> = Stepper::<2, spring::StaticSpringAndMass>::new(10000.0, 0.01);
	diff_equations::gui::main(stepper);
}

fn main() {
	create_image();
}