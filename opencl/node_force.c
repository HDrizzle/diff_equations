kernel void node_force (
	global ulong const* input,
	global ulong* output
) {
	const size_t i = get_global_id(0);
	output[i] = input[i] * input[i];
}