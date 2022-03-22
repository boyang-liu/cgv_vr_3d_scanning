#pragma once
#include <cgv/render/render_types.h>
#include <cgv/render/shader_program.h>
#include <3rd/glew/GL/glew.h>

#include <cgv/render/render_types.h>
#include "rgbd_depth.h"
#include <cgv_gl/sphere_renderer.h>
#include "Buffer.h"

class Reconstruction : public cgv::render::render_types {

public:
	Reconstruction() {
		
	};
	bool initialize(cgv::render::context& ctx);

	void deleteBuffers();

	void bindbuffer();

	void createBuffers();



protected:
	cgv::render::shader_program tsdf_prog;


	


private:

	Buffer gridpoints;






};