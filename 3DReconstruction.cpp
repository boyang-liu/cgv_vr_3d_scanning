#include "3DReconstruction.h"
#include <iostream>
#include <algorithm>

bool Reconstruction::initialize(cgv::render::context& ctx) {

	if (!tsdf_prog.build_program(ctx, "glsl/tsdf.glpr", true)) {
		std::cerr << "ERROR in building shader program " << std::endl;
		return false;
	}
	
	return true;
}
void Reconstruction::deleteBuffers() {

}
void Reconstruction::bindbuffer() {


}
void Reconstruction::createBuffers() {


}

