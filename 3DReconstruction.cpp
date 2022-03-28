#include "3DReconstruction.h"
#include <iostream>
#include <algorithm>

#define DPETHIMAGE_SSB_BP             0
#define VERTICES_SSB_BP             1


bool Reconstruction::build_program(cgv::render::context& ctx) {

	if (!tsdf_prog.build_program(ctx, "glsl/tsdf.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}
	
	return true;
}
bool Reconstruction::init(rgbd_depth input, vec3 min, vec3 max, ivec3 reso, float voxellength) {

	if (input.Pixels.size()==0)
		return false;

	currentdepthimage = input;
	min_pos= min;
	max_pos= max;
	resolution = reso;
	voxel_length = voxellength;
	return true;
}
bool Reconstruction::init(rgbd_depth input, vec3 min, vec3 max) {

	if (input.Pixels.size() == 0)
		return false;

	currentdepthimage = input;
	min_pos = min;
	max_pos = max;	
	return true;
}
bool Reconstruction::init(rgbd_depth input) {

	if (input.Pixels.size() == 0)
		return false;

	currentdepthimage = input;	
	return true;
}



bool Reconstruction::tsdf_algorithm(cgv::render::context& ctx) {

	if (currentdepthimage.Pixels.size() == 0)
		return false;
		
	ivec3 vereticesGridDims = ivec3(resolution[0] + 1, resolution[1] + 1, resolution[2] + 1);
	int length = vereticesGridDims[0] * vereticesGridDims[1] * vereticesGridDims[2];

	tsdf_prog.set_uniform(ctx, "vereticesGridDims", vereticesGridDims);
	tsdf_prog.set_uniform(ctx, "min_pos", min_pos);
	tsdf_prog.set_uniform(ctx, "max_pos", max_pos);
	tsdf_prog.set_uniform(ctx, "voxel_length", voxel_length);

	tsdf_prog.enable(ctx);
	glDispatchCompute(vereticesGridDims[0], vereticesGridDims[1], vereticesGridDims[2]);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	tsdf_prog.disable(ctx);

	return true;

}

bool Reconstruction::MC_algorithm(cgv::render::context& ctx) {

	return false;
}

bool Reconstruction::drawmesh(cgv::render::context& ctx) {

	return false;

}



void Reconstruction::deleteBuffers() {

	depthimage.deleteBuffer();
	vertices.deleteBuffer();

}
void Reconstruction::bindbuffer() {

	depthimage.setBindingPoint(DPETHIMAGE_SSB_BP);
	vertices.setBindingPoint(VERTICES_SSB_BP);

}





void Reconstruction::createBuffers() {

	int pixels_length = currentdepthimage.height * currentdepthimage.width;
	int vertices_length = (resolution[0] +1) * (resolution[1] + 1) * (resolution[2] + 1);
	

	depthimage = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, pixels_length * sizeof(int));
	vertices = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, vertices_length * sizeof(float));



}

