// GPU stuff
// Copied from https://github.com/kenba/opencl3/blob/main/examples/basic.rs

// Copyright (c) 2021 Via Technology Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use std::{ffi, ptr, fs};

use opencl3::command_queue::{CommandQueue, CL_QUEUE_PROFILING_ENABLE};
use opencl3::context::Context;
use opencl3::device::{get_all_devices, Device, CL_DEVICE_TYPE_GPU};
use opencl3::kernel::{ExecuteKernel, Kernel, self};
use opencl3::memory::{Buffer, CL_MEM_READ_ONLY, CL_MEM_WRITE_ONLY};
use opencl3::program::Program;
use opencl3::types::{cl_event, cl_float, cl_ulong, cl_double, cl_uint, CL_BLOCKING, CL_NON_BLOCKING};
use opencl3::Result;

use image::{RgbImage, Rgb, RgbaImage, ImageBuffer};

use crate::prelude::*;

//const KERNEL_NAME: &str = "texture_shader";
const OPENCL_SOURCE_DIR: &str = "opencl/";

pub struct GPUHandler {
	device: Device,
	context: Context,
	queue: CommandQueue
}

impl GPUHandler {
	pub fn new() -> Result<Self> {
		// Find a usable device for this application
		let device_id = *get_all_devices(CL_DEVICE_TYPE_GPU)?
		.first()
		.expect("no device found in platform");
		let device = Device::new(device_id);

		// Create a Context on an OpenCL device
		let context = Context::from_device(&device).expect("Context::from_device failed");

		// Create a command_queue on the Context's device
		let queue = CommandQueue::create_default(&context, CL_QUEUE_PROFILING_ENABLE)
			.expect("CommandQueue::create_default failed");
		Ok(Self {
			device,
			context,
			queue
		})
	}
	pub fn create_kernel(&self, program_name: &str) -> Result<Kernel> {
		// Build the OpenCL program source and create the kernel.
		let program_source = fs::read_to_string(format!("{}{}.c", OPENCL_SOURCE_DIR, program_name)).unwrap();
		let program = Program::create_and_build_from_source(&self.context, &program_source, "")
			.expect(&format!("Program::create_and_build_from_source failed when building source file \"{}\"", program_name));
		let kernel = Kernel::create(&program, program_name).expect("Kernel::create failed");
		// Done
		Ok(kernel)
	}
	pub fn render_texture(&self, state: &VDyn, image: &mut RgbImage, texture: &RgbaImage, translater: &ImagePosTranslater) {
		let kernel = self.create_kernel("texture_shader").unwrap();
		// TODO
	}
	pub fn test(&self, input: Vec<u64>) -> Result<Vec<u64>> {
		let kernel = self.create_kernel("test").unwrap();
		// Compute data

		// Create OpenCL device buffers
		let mut in_buffer = unsafe {
			Buffer::<cl_ulong>::create(&self.context, CL_MEM_READ_ONLY, input.len(), ptr::null_mut())?
		};
		let out_buffer = unsafe {
			Buffer::<cl_ulong>::create(&self.context, CL_MEM_WRITE_ONLY, input.len(), ptr::null_mut())?
		};
		println!("Created buffers");
		// Blocking write
		let _in_write_event = unsafe { self.queue.enqueue_write_buffer(&mut in_buffer, CL_BLOCKING, 0, &input, &[])? };
		println!("Wrote to in_buffer");
		// a value for the kernel function
		//let a: cl_float = 300.0;

		// Use the ExecuteKernel builder to set the kernel buffer and
		// cl_float value arguments, before setting the one dimensional
		// global_work_size for the call to enqueue_nd_range.
		// Unwraps the Result to get the kernel execution event.
		let kernel_event = unsafe {
			ExecuteKernel::new(&kernel)
				.set_arg(&in_buffer)
				.set_arg(&out_buffer)
				.set_global_work_size(input.len())
				//.set_wait_event(&y_write_event)
				.enqueue_nd_range(&self.queue)?
		};
		println!("Created kernel_event");
		let mut events: Vec<cl_event> = Vec::default();
		events.push(kernel_event.get());

		// Create a results array to hold the results from the OpenCL device
		// and enqueue a read command to read the device buffer into the array
		// after the kernel event completes.
		//let mut results: [cl_ulong; ARRAY_SIZE] = [0.0; ARRAY_SIZE];
		let mut results = Vec::<cl_ulong>::new();
		let read_event =
			unsafe { self.queue.enqueue_read_buffer(&out_buffer, CL_NON_BLOCKING, 0, &mut results, &events)? };
		println!("Created read_event");
		// Wait for the read_event to complete.
		read_event.wait()?;

		Ok(results)
	}
	pub fn test2_with_arrays_works(&self, input: Vec<u64>) -> Result<Vec<cl_ulong>> {
		/*// Find a usable device for this application
		let device_id = *get_all_devices(CL_DEVICE_TYPE_GPU)?
			.first()
			.expect("no device found in platform");
		let device = Device::new(device_id);

		// Create a Context on an OpenCL device
		let context = Context::from_device(&device).expect("Context::from_device failed");

		// Create a command_queue on the Context's device
		let queue = CommandQueue::create_default(&context, CL_QUEUE_PROFILING_ENABLE)
			.expect("CommandQueue::create_default failed");*/

		// Build the OpenCL program source and create the kernel.
		/*let program_name = "test".to_owned();
		let program_source = fs::read_to_string(format!("{}{}.c", OPENCL_SOURCE_DIR, &program_name)).unwrap();
		let program = Program::create_and_build_from_source(&self.context, &program_source, "")
			.expect("Program::create_and_build_from_source failed");
		let kernel = Kernel::create(&program, &program_name).expect("Kernel::create failed");*/
		let kernel = self.create_kernel("test").unwrap();


		/////////////////////////////////////////////////////////////////////
		// Compute data

		// The input data
		const ARRAY_SIZE: usize = 1000;
		let ones: [cl_ulong; ARRAY_SIZE] = [1; ARRAY_SIZE];
		let mut sums: [cl_ulong; ARRAY_SIZE] = [0; ARRAY_SIZE];
		for i in 0..ARRAY_SIZE {
			sums[i] = 1 + 1 * i as cl_ulong;
		}

		// Create OpenCL device buffers
		let mut x = unsafe {
			Buffer::<cl_ulong>::create(&self.context, CL_MEM_READ_ONLY, ARRAY_SIZE, ptr::null_mut())?
		};
		let mut y = unsafe {
			Buffer::<cl_ulong>::create(&self.context, CL_MEM_READ_ONLY, ARRAY_SIZE, ptr::null_mut())?
		};
		let z = unsafe {
			Buffer::<cl_ulong>::create(&self.context, CL_MEM_WRITE_ONLY, ARRAY_SIZE, ptr::null_mut())?
		};

		// Blocking write
		let _x_write_event = unsafe { self.queue.enqueue_write_buffer(&mut x, CL_BLOCKING, 0, &ones, &[])? };

		// Non-blocking write, wait for y_write_event
		let y_write_event =
			unsafe { self.queue.enqueue_write_buffer(&mut y, CL_NON_BLOCKING, 0, &sums, &[])? };

		// a value for the kernel function
		let a: cl_ulong = 300;

		// Use the ExecuteKernel builder to set the kernel buffer and
		// cl_float value arguments, before setting the one dimensional
		// global_work_size for the call to enqueue_nd_range.
		// Unwraps the Result to get the kernel execution event.
		let kernel_event = unsafe {
			ExecuteKernel::new(&kernel)
				.set_arg(&z)
				.set_arg(&x)
				.set_arg(&y)
				.set_arg(&a)
				.set_global_work_size(ARRAY_SIZE)
				.set_wait_event(&y_write_event)
				.enqueue_nd_range(&self.queue)?
		};

		let mut events: Vec<cl_event> = Vec::default();
		events.push(kernel_event.get());

		// Create a results array to hold the results from the OpenCL device
		// and enqueue a read command to read the device buffer into the array
		// after the kernel event completes.
		let mut results: [cl_ulong; ARRAY_SIZE] = [0; ARRAY_SIZE];
		let read_event =
			unsafe { self.queue.enqueue_read_buffer(&z, CL_NON_BLOCKING, 0, &mut results, &events)? };

		// Wait for the read_event to complete.
		read_event.wait()?;

		/*// Output the first and last results
		println!("results front: {}", results[0]);
		println!("results back: {}", results[ARRAY_SIZE - 1]);

		// Calculate the kernel duration, from the kernel_event
		let start_time = kernel_event.profiling_command_start()?;
		let end_time = kernel_event.profiling_command_end()?;
		let duration = end_time - start_time;
		println!("kernel execution duration (ns): {}", duration);*/

		Ok(results.iter().copied().collect())
	}
	pub fn test2_with_vecs_works(&self, input: Vec<u64>) -> Result<Vec<u64>> {
		let kernel = self.create_kernel("test").unwrap();


		/////////////////////////////////////////////////////////////////////
		// Compute data

		// The input data
		/*let ones = vec![1; input.len()];
		let mut sums = Vec::<u64>::new();
		for i in 0..input.len() {
			sums.push(1 + 1 * i as cl_ulong);
		}*/

		// Create OpenCL device buffers
		let mut input_buffer = unsafe {
			Buffer::<cl_ulong>::create(&self.context, CL_MEM_READ_ONLY, input.len(), ptr::null_mut())?
		};
		/*let mut y = unsafe {
			Buffer::<cl_ulong>::create(&self.context, CL_MEM_READ_ONLY, input.len(), ptr::null_mut())?
		};*/
		let output = unsafe {
			Buffer::<cl_ulong>::create(&self.context, CL_MEM_WRITE_ONLY, input.len(), ptr::null_mut())?
		};

		// Blocking write
		//let _input_write_event = unsafe { self.queue.enqueue_write_buffer(&mut input_buffer, CL_BLOCKING, 0, &input, &[])? };

		// Non-blocking write, wait for y_write_event
		let input_write_event =
			unsafe { self.queue.enqueue_write_buffer(&mut input_buffer, CL_NON_BLOCKING, 0, &input, &[])? };

		// a value for the kernel function
		//let a: cl_ulong = 300;

		// Use the ExecuteKernel builder to set the kernel buffer and
		// cl_float value arguments, before setting the one dimensional
		// global_work_size for the call to enqueue_nd_range.
		// Unwraps the Result to get the kernel execution event.
		let kernel_event = unsafe {
			ExecuteKernel::new(&kernel)
				.set_arg(&input_buffer)
				//.set_arg(&y)
				//.set_arg(&a)
				.set_arg(&output)
				.set_global_work_size(input.len())
				.set_wait_event(&input_write_event)
				.enqueue_nd_range(&self.queue)?
		};

		let mut events: Vec<cl_event> = Vec::default();
		events.push(kernel_event.get());

		// Create a results array to hold the results from the OpenCL device
		// and enqueue a read command to read the device buffer into the array
		// after the kernel event completes.
		let mut results = vec![0u64; input.len()];
		let read_event =
			unsafe { self.queue.enqueue_read_buffer(&output, CL_NON_BLOCKING, 0, &mut results, &events)? };

		// Wait for the read_event to complete.
		read_event.wait()?;

		/*// Output the first and last results
		println!("results front: {}", results[0]);
		println!("results back: {}", results[ARRAY_SIZE - 1]);

		// Calculate the kernel duration, from the kernel_event
		let start_time = kernel_event.profiling_command_start()?;
		let end_time = kernel_event.profiling_command_end()?;
		let duration = end_time - start_time;
		println!("kernel execution duration (ns): {}", duration);*/

		Ok(results)
	}
	/// Calculates acceleration on all of the nodes given a simulation state and various soft body settings
	pub fn node_acceleration(&self, input: Vec<u64>) -> Result<Vec<u64>> {
		let kernel = self.create_kernel("node_force").unwrap();
		// Create OpenCL device buffers
		let mut input_buffer = unsafe {
			Buffer::<cl_ulong>::create(&self.context, CL_MEM_READ_ONLY, input.len(), ptr::null_mut())?
		};
		/*let mut y = unsafe {
			Buffer::<cl_ulong>::create(&self.context, CL_MEM_READ_ONLY, input.len(), ptr::null_mut())?
		};*/
		let output = unsafe {
			Buffer::<cl_ulong>::create(&self.context, CL_MEM_WRITE_ONLY, input.len(), ptr::null_mut())?
		};

		// Blocking write
		//let _input_write_event = unsafe { self.queue.enqueue_write_buffer(&mut input_buffer, CL_BLOCKING, 0, &input, &[])? };

		// Non-blocking write, wait for y_write_event
		let input_write_event =
			unsafe { self.queue.enqueue_write_buffer(&mut input_buffer, CL_NON_BLOCKING, 0, &input, &[])? };

		// a value for the kernel function

		// Use the ExecuteKernel builder to set the kernel buffer and
		// cl_float value arguments, before setting the one dimensional
		// global_work_size for the call to enqueue_nd_range.
		// Unwraps the Result to get the kernel execution event.
		let kernel_event = unsafe {
			ExecuteKernel::new(&kernel)
				.set_arg(&input_buffer)
				//.set_arg(&y)
				//.set_arg(&a)
				.set_arg(&output)
				.set_global_work_size(input.len())
				.set_wait_event(&input_write_event)
				.enqueue_nd_range(&self.queue)?
		};

		let mut events: Vec<cl_event> = Vec::default();
		events.push(kernel_event.get());

		// Create a results array to hold the results from the OpenCL device
		// and enqueue a read command to read the device buffer into the array
		// after the kernel event completes.
		let mut results = vec![0u64; input.len()];
		let read_event =
			unsafe { self.queue.enqueue_read_buffer(&output, CL_NON_BLOCKING, 0, &mut results, &events)? };

		// Wait for the read_event to complete.
		read_event.wait()?;

		/*// Output the first and last results
		println!("results front: {}", results[0]);
		println!("results back: {}", results[ARRAY_SIZE - 1]);

		// Calculate the kernel duration, from the kernel_event
		let start_time = kernel_event.profiling_command_start()?;
		let end_time = kernel_event.profiling_command_end()?;
		let duration = end_time - start_time;
		println!("kernel execution duration (ns): {}", duration);*/

		Ok(results)
	}
	pub fn mandelbrot_test(
		&self,
		x: cl_double,
		y: cl_double,
		zoom: cl_double,
		width: cl_uint,
		height: cl_uint,
		max_iter: cl_uint
	) -> Result<Vec<u32>> {
		let kernel = self.create_kernel("mandelbrot").unwrap();

		let output_size = (width * height) as usize;
		// Create OpenCL device buffers
		/*let mut input_buffer = unsafe {
			Buffer::<cl_ulong>::create(&self.context, CL_MEM_READ_ONLY, input.len(), ptr::null_mut())?
		};
		let mut y = unsafe {
			Buffer::<cl_ulong>::create(&self.context, CL_MEM_READ_ONLY, input.len(), ptr::null_mut())?
		};*/
		let output = unsafe {
			Buffer::<cl_uint>::create(&self.context, CL_MEM_WRITE_ONLY, output_size, ptr::null_mut())?
		};

		// Blocking write
		//let _input_write_event = unsafe { self.queue.enqueue_write_buffer(&mut input_buffer, CL_BLOCKING, 0, &input, &[])? };

		// Non-blocking write, wait for y_write_event
		//let input_write_event = unsafe { self.queue.enqueue_write_buffer(&mut input_buffer, CL_NON_BLOCKING, 0, &input, &[])? };

		// a value for the kernel function
		//let a: cl_ulong = 300;

		// Use the ExecuteKernel builder to set the kernel buffer and
		// cl_float value arguments, before setting the one dimensional
		// global_work_size for the call to enqueue_nd_range.
		// Unwraps the Result to get the kernel execution event.
		let kernel_event = unsafe {
			ExecuteKernel::new(&kernel)
				.set_arg(&x)
				.set_arg(&y)
				.set_arg(&zoom)
				.set_arg(&width)
				.set_arg(&height)
				.set_arg(&max_iter)
				.set_arg(&output)
				.set_global_work_size(output_size)
				//.set_wait_event(&input_write_event)
				.enqueue_nd_range(&self.queue)?
		};

		let mut events: Vec<cl_event> = Vec::default();
		events.push(kernel_event.get());

		// Create a results array to hold the results from the OpenCL device
		// and enqueue a read command to read the device buffer into the array
		// after the kernel event completes.
		let mut results = vec![0u32; output_size];
		let read_event =
			unsafe { self.queue.enqueue_read_buffer(&output, CL_NON_BLOCKING, 0, &mut results, &events)? };

		// Wait for the read_event to complete.
		read_event.wait()?;

		/*// Output the first and last results
		println!("results front: {}", results[0]);
		println!("results back: {}", results[ARRAY_SIZE - 1]);

		// Calculate the kernel duration, from the kernel_event
		let start_time = kernel_event.profiling_command_start()?;
		let end_time = kernel_event.profiling_command_end()?;
		let duration = end_time - start_time;
		println!("kernel execution duration (ns): {}", duration);*/

		Ok(results)
	}
}

pub fn mandelbrot_test() {
	// mandelbrot args
	let x = 0.0;
	let y = 0.0;
	let zoom = 50.0;
	let width = 200;
	let height = 200;
	let max_iter = 200;
	// Misc
	let background = Rgb([0; 3]);
	let iter_to_px = |iter: u32| -> Rgb<u8> {
		if iter == max_iter {
			Rgb([0; 3])
		}
		else {
			let frac = iter as f32 / max_iter as f32;
			Rgb([(255 as f32 * frac) as u8; 3])
		}
	};
	// Compute
	let gpu_handler = GPUHandler::new().unwrap();
	let results = gpu_handler.mandelbrot_test(x, y, zoom, width, height, max_iter).unwrap();
	// Create image
	let mut image: RgbImage = ImageBuffer::from_pixel(width, height, background);
	for x in 0..width {
		for y in 0..height {
			image.put_pixel(x, y, iter_to_px(results[(y * width + x) as usize]));
		}
	}
	// Done
	image.save("mandelbrot.png");
}

#[cfg(test)]
mod tests {
    use super::*;
	#[test]
	fn vector_math() {
		let gpu_handler = GPUHandler::new().unwrap();
		let input = Vec::<u64>::from_iter(0..10);
		let output = gpu_handler.test2_with_vecs_works(input).unwrap();
		for (i, out_n) in output.iter().enumerate() {
			println!("{}: {}", i, out_n);
		}
	}
}

/*fn main() -> Result<()> {
	// Find a usable device for this application
	let device_id = *get_all_devices(CL_DEVICE_TYPE_GPU)?
		.first()
		.expect("no device found in platform");
	let device = Device::new(device_id);

	// Create a Context on an OpenCL device
	let context = Context::from_device(&device).expect("Context::from_device failed");

	// Create a command_queue on the Context's device
	let queue = CommandQueue::create_default(&context, CL_QUEUE_PROFILING_ENABLE)
		.expect("CommandQueue::create_default failed");

	// Build the OpenCL program source and create the kernel.
	let program = Program::create_and_build_from_source(&context, PROGRAM_SOURCE, "")
		.expect("Program::create_and_build_from_source failed");
	let kernel = Kernel::create(&program, KERNEL_NAME).expect("Kernel::create failed");

	/////////////////////////////////////////////////////////////////////
	// Compute data

	// The input data
	const ARRAY_SIZE: usize = 1000;
	let ones: [cl_float; ARRAY_SIZE] = [1.0; ARRAY_SIZE];
	let mut sums: [cl_float; ARRAY_SIZE] = [0.0; ARRAY_SIZE];
	for i in 0..ARRAY_SIZE {
		sums[i] = 1.0 + 1.0 * i as cl_float;
	}

	// Create OpenCL device buffers
	let mut x = unsafe {
		Buffer::<cl_float>::create(&context, CL_MEM_READ_ONLY, ARRAY_SIZE, ptr::null_mut())?
	};
	let mut y = unsafe {
		Buffer::<cl_float>::create(&context, CL_MEM_READ_ONLY, ARRAY_SIZE, ptr::null_mut())?
	};
	let z = unsafe {
		Buffer::<cl_float>::create(&context, CL_MEM_WRITE_ONLY, ARRAY_SIZE, ptr::null_mut())?
	};

	// Blocking write
	let _x_write_event = unsafe { queue.enqueue_write_buffer(&mut x, CL_BLOCKING, 0, &ones, &[])? };

	// Non-blocking write, wait for y_write_event
	let y_write_event =
		unsafe { queue.enqueue_write_buffer(&mut y, CL_NON_BLOCKING, 0, &sums, &[])? };

	// a value for the kernel function
	let a: cl_float = 300.0;

	// Use the ExecuteKernel builder to set the kernel buffer and
	// cl_float value arguments, before setting the one dimensional
	// global_work_size for the call to enqueue_nd_range.
	// Unwraps the Result to get the kernel execution event.
	let kernel_event = unsafe {
		ExecuteKernel::new(&kernel)
			.set_arg(&z)
			.set_arg(&x)
			.set_arg(&y)
			.set_arg(&a)
			.set_global_work_size(ARRAY_SIZE)
			.set_wait_event(&y_write_event)
			.enqueue_nd_range(&queue)?
	};

	let mut events: Vec<cl_event> = Vec::default();
	events.push(kernel_event.get());

	// Create a results array to hold the results from the OpenCL device
	// and enqueue a read command to read the device buffer into the array
	// after the kernel event completes.
	let mut results: [cl_float; ARRAY_SIZE] = [0.0; ARRAY_SIZE];
	let read_event =
		unsafe { queue.enqueue_read_buffer(&z, CL_NON_BLOCKING, 0, &mut results, &events)? };

	// Wait for the read_event to complete.
	read_event.wait()?;

	// Output the first and last results
	println!("results front: {}", results[0]);
	println!("results back: {}", results[ARRAY_SIZE - 1]);

	// Calculate the kernel duration, from the kernel_event
	let start_time = kernel_event.profiling_command_start()?;
	let end_time = kernel_event.profiling_command_end()?;
	let duration = end_time - start_time;
	println!("kernel execution duration (ns): {}", duration);

	Ok(())
}*/