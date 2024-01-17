// Most functionality is in lib.rs, this is only for playing around
#![allow(warnings)]
use diff_equations::{prelude::*, physics};
use nalgebra::max;
use image::{Rgb, RgbImage, ImageBuffer};

fn create_image() {
	let background = Rgb([0; 3]);
	let fill_color = Rgb([255; 3]);
	let num_iterations = 2000;
	let max_magnitude = 10000.0;
	let step = 0.005;
	let scale = 200.0;
	let image_size = ImgV2::new(500, 500);
	let origin = V2::zeros();
	let mut stepper: Stepper<StaticSpringAndMass> = Stepper::<StaticSpringAndMass>::new(StaticSpringAndMass::default(), max_magnitude, step);
	stepper.state = VDyn::from_vec(vec![0.0, 1.0]);
	let mut image: RgbImage = ImageBuffer::from_pixel(image_size.x, image_size.y, background);
	// Render
	render_image(
		stepper,
		&PlotVariableIndices(vec![0,1]),
		&mut image,
		ImagePosTranslater {
			scale,
			origin,
			image_size
		},
		num_iterations,
		fill_color
	);
	// Save
	image.save("phase_portrait.png");
}

/*fn start_gui() {
	let stepper: Stepper<StaticSpringAndMass> = Stepper::<StaticSpringAndMass>::new(StaticSpringAndMass::default(), 10000.0, 0.01);
	diff_equations::gui::main(stepper);
}*/

fn debug_print() {
	let num_iterations = 10;
	let step = 0.05;
	let max_magnitude = 10000.0;
	let mut stepper: Stepper<StaticSpringAndMass> = Stepper::<StaticSpringAndMass>::new(StaticSpringAndMass::default(), max_magnitude, step);
	stepper.state = VDyn::from_vec(vec![0.0, 1.0]);
	for i in 0..num_iterations {
		println!("{}: {:?}", i, stepper.state);
		stepper.step();
	}
}

fn main() {
	//soft_body::test();
	physics::speeed::mandelbrot_test();
}