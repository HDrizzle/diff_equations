kernel void mandelbrot (
	double x,// f64
	double y,// f64
	double zoom,// f64
	uint width,// u32
	uint height,// u32
	uint max_iter,// u32
	global uint* output// [u32; width * height]
) {
	const size_t i = get_global_id(0);
	output[i] = max_iter / 2;// TODO
}