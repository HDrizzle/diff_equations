// Original example
/*kernel void saxpy_float (
	global float* z,
	global float const* x,
	global float const* y,
	float a
) {
	const size_t i = get_global_id(0);
	z[i] = a*x[i] + y[i];
}*/

kernel void texture_shader (
	__read_only image2d_t* texture,
	__read_write image2d_t* output_image
) {
	const size_t i = get_global_id(0);
	// Just testing for now, in reality the whole point of this is that these for-loops will not be needed
	for (int x = 0; x < get_image_width(output_image); x++) {
		for (int y = 0; x < get_image_height(output_image); y++) {
			// TODO
		}
	}
}