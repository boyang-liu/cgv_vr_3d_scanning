#include "3DReconstruction.h"
#include <iostream>
#include <algorithm>

#define DPETHIMAGE_SSB_BP             0
#define VERTICES_SSB_BP             1


bool Reconstruction::initialize(cgv::render::context& ctx) {

	if (!tsdf_prog.build_program(ctx, "glsl/tsdf.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}
	
	return true;
}
bool Reconstruction::init(rgbd_depth input,vec3 min,vec3 max, int reso ) {

	if (input.Pixels.size()==0)
		return false;

	currentdepthimage = input;
	min_pos= min;
	max_pos= max;
	resolution = reso;
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
	
	int length = (resolution + 1) * (resolution + 1) * (resolution + 1);

	tsdf_prog.set_uniform(ctx, "resolution", resolution);
	tsdf_prog.set_uniform(ctx, "min_pos", min_pos);
	tsdf_prog.set_uniform(ctx, "max_pos", max_pos);
	tsdf_prog.enable(ctx);
	glDispatchCompute(resolution + 1, resolution + 1, resolution + 1);
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
	int vertices_length = (resolution+1) * (resolution + 1) * (resolution + 1);
	int pixels_length = currentdepthimage.height * currentdepthimage.width;

	depthimage = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, pixels_length * sizeof(int));
	vertices = Buffer(GL_SHADER_STORAGE_BUFFER, GL_DYNAMIC_COPY, vertices_length * sizeof(float));



}

