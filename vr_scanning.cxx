﻿#include "vr_scanning.h"

/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
void vr_scanning::construct_table(float tw, float td, float th, float tW)
{
	// construct table
	rgb table_clr(0.3f, 0.2f, 0.0f);
	boxes.push_back(box3(vec3(-0.5f * tw - 2 * tW, th, -0.5f * td - 2 * tW),
		vec3(0.5f * tw + 2 * tW, th + tW, 0.5f * td + 2 * tW)));
	box_colors.push_back(table_clr);

	boxes.push_back(box3(vec3(-0.5f * tw, 0, -0.5f * td), vec3(-0.5f * tw - tW, th, -0.5f * td - tW)));
	boxes.push_back(box3(vec3(-0.5f * tw, 0, 0.5f * td), vec3(-0.5f * tw - tW, th, 0.5f * td + tW)));
	boxes.push_back(box3(vec3(0.5f * tw, 0, -0.5f * td), vec3(0.5f * tw + tW, th, -0.5f * td - tW)));
	boxes.push_back(box3(vec3(0.5f * tw, 0, 0.5f * td), vec3(0.5f * tw + tW, th, 0.5f * td + tW)));
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
}
/// construct boxes that represent a room of dimensions w,d,h and wall width W
void vr_scanning::construct_room(float w, float d, float h, float W, bool walls, bool ceiling)
{
	// construct floor
	boxes.push_back(box3(vec3(-0.5f * w, -W, -0.5f * d), vec3(0.5f * w, 0, 0.5f * d)));
	box_colors.push_back(rgb(0.2f, 0.2f, 0.2f));

	if (walls) {
		// construct walls
		boxes.push_back(box3(vec3(-0.5f * w, -W, -0.5f * d - W), vec3(0.5f * w, h, -0.5f * d)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));
		boxes.push_back(box3(vec3(-0.5f * w, -W, 0.5f * d), vec3(0.5f * w, h, 0.5f * d + W)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));

		boxes.push_back(box3(vec3(0.5f * w, -W, -0.5f * d - W), vec3(0.5f * w + W, h, 0.5f * d + W)));
		box_colors.push_back(rgb(0.5f, 0.8f, 0.5f));
	}
	if (ceiling) {
		// construct ceiling
		boxes.push_back(box3(vec3(-0.5f * w - W, h, -0.5f * d - W), vec3(0.5f * w + W, h + W, 0.5f * d + W)));
		box_colors.push_back(rgb(0.5f, 0.5f, 0.8f));
	}
}

/// construct boxes for environment
void vr_scanning::construct_environment(float s, float ew, float ed, float eh, float w, float d, float h)
{
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	unsigned n = unsigned(ew / s);
	unsigned m = unsigned(ed / s);
	for (unsigned i = 0; i < n; ++i) {
		float x = i * s - 0.5f * ew;
		for (unsigned j = 0; j < m; ++j) {
			float z = j * s - 0.5f * ed;
			if ((x + s > -0.5f * w && x < 0.5f * w) && (z + s > -0.5f * d && z < 0.5f * d))
				continue;
			float h = 0.2f * (std::max(abs(x) - 0.5f * w, 0.0f) + std::max(abs(z) - 0.5f * d, 0.0f)) *
				distribution(generator) +
				0.1f;
			boxes.push_back(box3(vec3(x, 0, z), vec3(x + s, h, z + s)));
			box_colors.push_back(rgb(0.3f * distribution(generator) + 0.3f, 0.3f * distribution(generator) + 0.2f,
				0.2f * distribution(generator) + 0.1f));
		}
	}
}

/// construct boxes that can be moved around
void vr_scanning::construct_movable_boxes(float tw, float td, float th, float tW, size_t nr)
{
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	std::uniform_real_distribution<float> signed_distribution(-1, 1);
	for (size_t i = 0; i < nr; ++i) {
		float x = distribution(generator);
		float y = distribution(generator);
		vec3 extent(distribution(generator), distribution(generator), distribution(generator));
		extent += 0.1f;
		extent *= std::min(tw, td) * 0.2f;

		vec3 center(-0.5f * tw + x * tw, th + tW, -0.5f * td + y * td);
		movable_boxes.push_back(box3(-0.5f * extent, 0.5f * extent));
		movable_box_colors.push_back(rgb(distribution(generator), distribution(generator), distribution(generator)));
		movable_box_translations.push_back(center);
		quat rot(signed_distribution(generator), signed_distribution(generator), signed_distribution(generator),
			signed_distribution(generator));
		rot.normalize();
		movable_box_rotations.push_back(rot);
	}
}


// compute intersection points of controller ray with movable boxes
void vr_scanning::compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color)
	{
		for (size_t i = 0; i < movable_boxes.size(); ++i) {
			vec3 origin_box_i = origin - movable_box_translations[i];
			movable_box_rotations[i].inverse_rotate(origin_box_i);
			vec3 direction_box_i = direction;
			movable_box_rotations[i].inverse_rotate(direction_box_i);
			float t_result;
			vec3 p_result;
			vec3 n_result;
			if (cgv::media::ray_axis_aligned_box_intersection(origin_box_i, direction_box_i, movable_boxes[i], t_result,
															  p_result, n_result, 0.000001f))
			{

				// transform result back to world coordinates
				movable_box_rotations[i].rotate(p_result);
				p_result += movable_box_translations[i];
				movable_box_rotations[i].rotate(n_result);

				// store intersection information
				intersection_points.push_back(p_result);
				intersection_colors.push_back(color);
				intersection_box_indices.push_back((int)i);
				intersection_controller_indices.push_back(ci);
			}
		}
	}

/// register on device change events
void vr_scanning::on_device_change(void* kit_handle, bool attach)
{
	post_recreate_gui();
}
/// construct a scene with a table
void vr_scanning::build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW)
{
	construct_room(w, d, h, W, false, false);
	construct_table(tw, td, th, tW);
	construct_environment(0.2f, 3 * w, 3 * d, h, w, d, h);
	construct_movable_boxes(tw, td, th, tW, 20);
}
	/// generate a random point cloud
	void vr_scanning::generate_point_cloud(point_cloud& pc)
	{
		/*std::default_random_engine r;
		std::uniform_real_distribution<float> d(0.0f, 1.0f);
		vec3 S(0.0f, 2.0f, 0.0f);
		vec3 V(1.0f, 0, 0);
		vec3 U(0.0f, 1.0f, 0);
		vec3 X = cross(V, U);
		float aspect = 1.333f;
		float tan_2 = 0.3f;
		for (int i = 0; i < 10000; ++i) {
			float x = 2 * d(r) - 1;
			float y = 2 * d(r) - 1;
			float z = d(r) + 1;
			vec3 p = x * aspect * tan_2 * z * X + y * tan_2 * z * U + z * V;
			rgba8 c((cgv::type::uint8_type)(255 * d(r)), 0, 0);
			vertex v;
			v.point = S + p;
			v.color = c;
			pc.push_back(v);
		}*/
	}
	/// start the rgbd device
void vr_scanning::start_rgbd()
{
		if (!rgbd_inp.is_attached()) {
			if (rgbd::rgbd_input::get_nr_devices() == 0)
				return;		
			if (!rgbd_inp.attach(rgbd::rgbd_input::get_serial(0)))
				return;
		}
		rgbd_inp.set_near_mode(true);
		std::vector<rgbd::stream_format> stream_formats;
		rgbd_started = rgbd_inp.start(rgbd::IS_COLOR_AND_DEPTH, stream_formats);
		update_member(&rgbd_started);
}
/// stop rgbd device
void vr_scanning::stop_rgbd()
	{
		if (!rgbd_inp.is_started())
			return;
		rgbd_started = rgbd_inp.stop();
		update_member(&rgbd_started);
	}

vr_scanning::vr_scanning()
{
		set_name("vr_scanning");
		rgbd_controller_index = 0;
		controller_orientation.identity();
		controller_position = vec3(0, 1.5f, 0);
		in_calibration = false;
		zoom_in = false;
		zoom_out = false;
		save_pointcloud = true;
		registration_started = false;
		rgbd_2_controller_orientation.identity();
		rgbd_2_controller_orientation.set_col(0, vec3(-1, 0, 0));
		rgbd_2_controller_orientation.set_col(1, vec3(0, -0.7071f, 0.7071f));
		rgbd_2_controller_orientation.set_col(2, vec3(0, 0.7071f, 0.7071f));
		rgbd_2_controller_position.zeros();

		rgbd_2_controller_orientation_start_calib.identity();
		rgbd_2_controller_position_start_calib.zeros();

		build_scene(5, 7, 3, 0.2f, 1.6f, 0.8f, 0.9f, 0.03f);
		// icp_pc->read("C:/Users/ltf/Desktop/test/monkey.obj");
		// pc2vertex(*icp_pc, current_pc);
		// generate_point_cloud(current_pc);
		vr_view_ptr = 0;
		ray_length = 2;
		connect(cgv::gui::ref_vr_server().on_device_change, this, &vr_scanning::on_device_change);

		srs.radius = 0.005f;
		ars.length_scale = 0.03f;
		state[0] = state[1] = state[2] = state[3] = InteractionState::IS_NONE;
		rgbd_started = false;
		record_frame = false;
		record_all_frames = false;
		record_key_frames = false;
		clear_all_frames = false;
		trigger_is_pressed = false;
		recording_fps = 5;
		show_points = true;
		point_style.point_size = 2;
		point_style.blend_points = false;
		point_style.blend_width_in_pixel = 0;
		max_nr_shown_recorded_pcs = 20;
		counter_pc = 0;
		remap_color = true;
		is_filled = false;
		is_filled_2 = false;

		pc_file_path = QUOTE_SYMBOL_VALUE(INPUT_DIR) " / .. / data";

		connect(cgv::gui::get_animation_trigger().shoot, this, &vr_scanning::timer_event);
}







size_t vr_scanning::generate_depths() {
	intermediate_depth.clear_depth();
	const unsigned short* depths = reinterpret_cast<const unsigned short*>(&depth_frame_2.frame_data.front());

	double fx_d, fy_d, cx_d, cy_d, depth_scale;
	rgbd_inp.get_V1_parameters(fx_d, fy_d, cx_d, cy_d, depth_scale);
	//std::cout<< fx_d <<"/" << fy_d << "/" << cx_d << "/" << cy_d << "/" << depth_scale << std::endl;

	
	intermediate_depth.para.fx_d = fx_d;
	intermediate_depth.para.fy_d = fy_d;
	intermediate_depth.para.cx_d = cx_d;
	intermediate_depth.para.cy_d = cy_d;
	intermediate_depth.para.depth_scale = depth_scale;

	intermediate_depth.height = depth_frame_2.height;
	intermediate_depth.width = depth_frame_2.width;



	std::vector<int> mypixels;

	int i = 0;
	for (int y = 0; y < depth_frame_2.height; ++y)
		for (int x = 0; x < depth_frame_2.width; ++x) {
			
			mypixels.push_back(depths[i]);

			++i;

		}
	intermediate_depth.Pixels = mypixels;

	//return intermediate_depth.get_nr_points();
	return 0;
}

void vr_scanning::generate_mesh() {




}








size_t vr_scanning::construct_point_cloud()
{
	/*double fx_d, fy_d, cx_d, cy_d, depth_scale;
	rgbd_inp.get_V1_parameters(fx_d, fy_d, cx_d, cy_d, depth_scale);
	std::cout<< fx_d << fy_d << cx_d << cy_d << depth_scale <<std::endl;*/




		intermediate_pc.clear();
		const unsigned short* depths = reinterpret_cast<const unsigned short*>(&depth_frame_2.frame_data.front());
		const unsigned char* colors = reinterpret_cast<const unsigned char*>(&color_frame_2.frame_data.front());

		if (remap_color) {
			rgbd_inp.map_color_to_depth(depth_frame_2, color_frame_2, warped_color_frame_2);
			if (trigger_is_pressed)
			{
				//src_img = cv::Mat(warped_color_frame_2.width, warped_color_frame_2.height, CV_32FC4, reinterpret_cast<void*>(&warped_color_frame_2.frame_data.front()));
				//cv::Mat tgt_img = cv::Mat(warped_color_frame_2.width, warped_color_frame_2.height, CV_32FC4, reinterpret_cast<void*>(&recorded_frames.back().frame_data.front()));
				//std::cout << "1111111111" << std::endl;
				//cv::imshow("456", src_img);
				//recorded_frames.emplace_back(color_frame_2);
			}

			//TODO we should extract FAST10 or ORB feature from warped_color_frame, the feature points sould be assign as RED
			colors = reinterpret_cast<const unsigned char*>(&warped_color_frame_2.frame_data.front());
		}

		int i = 0;
		for (int y = 0; y < depth_frame_2.height; ++y)
			for (int x = 0; x < depth_frame_2.width; ++x) {
				vec3 p;
				if (rgbd_inp.map_depth_to_point(x, y, depths[i], &p[0])) {
					// flipping y to make it the same direction as in pixel y coordinate
					p = -p;
					//p = rgbd_2_controller_orientation * p + rgbd_2_controller_position;
					//p = controller_orientation_pc * p + controller_position_pc;
					rgba8 c(colors[4 * i + 2], colors[4 * i + 1], colors[4 * i], 255);
					point_cloud::Pnt vp;
					point_cloud::Clr vc;
					// filter points without color for 32 bit formats
					static const rgba8 filter_color = rgba8(0, 0, 0, 255);
					if (!(c == filter_color)) {
						vc = c;
						vp = p;
					}
					// v.point = p;
					// v.color = c;
					intermediate_pc.add_point(vp, vc);
				}
				++i;
			}
		return intermediate_pc.get_nr_points();
}
rgbd::frame_type vr_scanning::read_rgb_frame() // should be a thread
{
	return color_frame;
}
rgbd::frame_type vr_scanning::read_depth_frame()
{
	return depth_frame;
}
/// here should be const point cloud
void vr_scanning::write_pcs_to_disk(int i)
{
		/*if (!intermediate_pc.empty()) {
			// define point cloud type, wirte to disk
			point_cloud* pc_save = new point_cloud();
			pc_save->has_clrs = true;
			copy_pointcloud(intermediate_pc, *pc_save);
			/// pathname
			std::string filename = pc_file_path + std::to_string(i) + ".obj";
			pc_save->write(filename);
		}*/
}
void vr_scanning::read_pc_queue(const std::string filename, std::string content)
{
	cgv::utils::file::read(filename, content, false);
	// read pcs from disk
}
void vr_scanning::registrationPointCloud()
{
		cgv::pointcloud::ICP* icp = new cgv::pointcloud::ICP();
		if (recorded_pcs.size() >= 1) {
			cgv::math::fmat<float, 3, 3> r;
			cgv::math::fvec<float, 3> t;
			r.identity();
			t.zeros();
			/*point_cloud* sourcePC = new point_cloud();
			point_cloud* sourcecopy = new point_cloud();
			point_cloud* targetPC = new point_cloud();
			sourcePC->resize(intermediate_pc.size());
			targetPC->resize(recorded_pcs.front().size());
			sourcecopy->resize(intermediate_pc.size());
			copy_pointcloud(recorded_pcs.front(), *targetPC);
			copy_pointcloud(intermediate_pc, *sourcePC);*/
			icp->set_source_cloud(intermediate_pc);
			icp->set_target_cloud(recorded_pcs.front());
			icp->set_iterations(5);
			icp->set_eps(1e-10);
			//icp->reg_icp(r, t);
			icp->get_crspd(r, t, *icp->crspd_source, *icp->crspd_target);
			for (int i = 0; i < intermediate_pc.get_nr_points(); i++) {
				intermediate_pc.pnt(i) = r * intermediate_pc.pnt(i) + t;
			}
			//intermediate_pc.clear();
			//pc2vertex(*sourcePC, intermediate_pc);
			//std::cout << "size: " << recorded_pcs.size() << " " << intermediate_pc.size() << std::endl;
		}
}

void vr_scanning::generate_rdm_pc(point_cloud& pc1, point_cloud& pc2)
{
		mat3 rotate_m;
		rotate_m.identity();
		double theta = M_PI / 8; // The angle of rotation in radians
		rotate_m.set_col(0, vec3(std::cos(theta), -sin(theta), 0));
		rotate_m.set_col(1, vec3(sin(theta), std::cos(theta), 0));
		rotate_m.set_col(2, vec3(0, 0, 1));
		for (int i = 0; i < 10000; i++) {
			point_cloud_types::Pnt origin;
			origin.zeros();
			origin.x() = 1024 * rand() / (RAND_MAX + 1.0f);
			origin.y() = 1024 * rand() / (RAND_MAX + 1.0f);
			origin.z() = 1024 * rand() / (RAND_MAX + 1.0f);
			pc1.pnt(i) = origin;
			origin = rotate_m * origin;
			pc2.pnt(i) = origin + vec3(0.0, 0.4, 0.4);
		}
}

void vr_scanning::test_icp()
{
		cgv::pointcloud::ICP* icp = new cgv::pointcloud::ICP();
		cgv::math::fmat<float, 3, 3> r;
		cgv::math::fvec<float, 3> t;
		r.identity();
		t.zeros();
		point_cloud* sourcePC = new point_cloud();
		point_cloud* targetPC = new point_cloud();
		sourcePC->resize(10000);
		targetPC->resize(10000);
		generate_rdm_pc(*sourcePC, *targetPC);
		icp->set_source_cloud(*sourcePC);
		icp->set_target_cloud(*targetPC);
		// icp->set_source_cloud(*targetPC);
		// icp->set_target_cloud(*sourcePC);
		icp->set_iterations(5);
		icp->set_num_random(3);
		icp->set_eps(1e-10);
		icp->reg_icp(r, t);
}

void vr_scanning::construct_TSDtree()
{
		// using pc queue to construct the TSDtree
}
bool vr_scanning::record_this_frame(double t)
{
		if (!(record_frame || record_all_frames || trigger_is_pressed))
			return false;
		static double last_recording_time = -1;
		if (t - last_recording_time < 1.0 / recording_fps)
			return false;
		last_recording_time = t;
		return true;
}

void vr_scanning::timer_event(double t, double dt)
{
		// in case a point cloud is being constructed
		if (future_handle.valid()) {
			// check for termination of thread
			if (future_handle.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
				size_t N = future_handle.get();
				// copy computed point cloud
				if (record_this_frame(t)) {
					if (registration_started && !recorded_pcs.empty()) {
						registrationPointCloud();
						// test_icp();
						// on_reg_SICP_cb();
						registration_started = !registration_started;
						update_member(&registration_started);
					}
					recorded_pcs.emplace_back(intermediate_pc);
					if (save_pointcloud) {
						counter_pc++;
						write_pcs_to_disk(counter_pc);
					}
					current_pc.clear();
					record_frame = false;
					update_member(&record_frame);
				}
				else if (clear_all_frames) {
					recorded_pcs.clear();
					recorded_frames.clear();
					clear_all_frames = false;
					update_member(&clear_all_frames);
				}
				else
					current_pc = intermediate_pc;
				post_redraw();
			}
		}
		if (rgbd_inp.is_started()) {
			if (rgbd_inp.is_started()) {
				bool new_frame;
				bool found_frame = false;
				bool depth_frame_changed = false;
				bool color_frame_changed = false;
				do {
					new_frame = false;
					bool new_color_frame_changed = rgbd_inp.get_frame(rgbd::IS_COLOR, color_frame, 0);
					//std::cout << "image_size: " << color_frame.height << " " << color_frame.width << " " << color_frame.frame_data.size() << std::endl;
					// TODO add ORB feature extraction and estimate camera pose for the warpped color images
					if (new_color_frame_changed) {
						++nr_color_frames;
						color_frame_changed = new_color_frame_changed;
						new_frame = true;
						/*if (trigger_is_pressed) {
							src_img = cv::Mat(color_frame.height, color_frame.width, CV_8UC4, static_cast<void*>(&color_frame.frame_data.front()));
							record_imgs.emplace_back(src_img);
							std::cout << "record " << record_imgs.size() << std::endl;
						}	
						update_member(&nr_color_frames);*/
					}
					bool new_depth_frame_changed = rgbd_inp.get_frame(rgbd::IS_DEPTH, depth_frame, 0);
					if (new_depth_frame_changed) {
						++nr_depth_frames;
						depth_frame_changed = new_depth_frame_changed;
						new_frame = true;
						update_member(&nr_depth_frames);
					}
					if (new_frame)
						found_frame = true;
				} while (new_frame);
				if (found_frame)
					post_redraw();
				if (color_frame.is_allocated() && depth_frame.is_allocated() &&
					(color_frame_changed || depth_frame_changed)) {

					if (!future_handle.valid()) {
						/*if (!in_calibration) {
							color_frame_2 = color_frame;
							depth_frame_2 = depth_frame;
						}
						if (zoom_out && !zoom_in) {
							controller_orientation_pc = controller_orientation * 2;
							controller_position_pc = controller_position;
						}
						else if (zoom_in && !zoom_out) {
							controller_orientation_pc = controller_orientation * 0.5;
							controller_position_pc = controller_position;
						}
						else {
							controller_orientation_pc = controller_orientation;
							controller_position_pc = controller_position;
						}*/
						color_frame_2 = color_frame;
						depth_frame_2 = depth_frame;

						//controller_orientation_pc = controller_orientation;
						//controller_position_pc = controller_position;
						future_handle = std::async(&vr_scanning::construct_point_cloud, this);
						//generate_depths();
					}
				}
			}
		}
}

std::string vr_scanning::get_type_name() const
{
		return "vr_scanning";
}
void vr_scanning::create_gui()
{
		add_decorator("vr_rgbd", "heading", "level=2");

		/*connect_copy(add_control("device", (DummyEnum&)device_idx, "dropdown", device_def)->value_change, rebind(this, &vr_scanning::on_device_select_cb));
		add_member_control(this, "near_mode", near_mode, "toggle");
		add_member_control(this, "remap_color", remap_color, "toggle");
		add_member_control(this, "flip x", flip[0], "toggle", "w=66", " ");
		add_member_control(this, "flip y", flip[1], "toggle", "w=66", " ");
		add_member_control(this, "flip z", flip[2], "toggle", "w=66");
		add_view("nr_color_frames", nr_color_frames);
		add_view("nr_infrared_frames", nr_infrared_frames);
		add_view("nr_depth_frames", nr_depth_frames);
		add_view("nr_mesh_frames", nr_mesh_frames);*/

		add_gui("rgbd_protocol_path", rgbd_protocol_path, "directory", "w=150");
		add_member_control(this, "rgbd_started", rgbd_started, "check");
		add_member_control(this, "record_frame", record_frame, "check");
		add_member_control(this, "record_all_frames", record_all_frames, "check");
		add_member_control(this, "clear_all_frames", clear_all_frames, "check");
		add_member_control(this, "trigger_is_pressed", trigger_is_pressed, "check");
		add_member_control(this, "recording_fps", recording_fps, "value_slider", "min=1;max=30;ticks=true;log=true");
		add_member_control(this, "in_calibration", in_calibration, "check");
		add_member_control(this, "zoom_in", zoom_in, "check");
		add_member_control(this, "zoom_out", zoom_out, "check");
		add_member_control(this, "save_pc", save_pointcloud, "check");
		add_member_control(this, "register_pc", registration_started, "check");
		connect_copy(add_button("opencv", "shortcut='o'")->click, rebind(this, &vr_scanning::open_image));
		connect_copy(add_button("is_filled", "shortcut='o'")->click, rebind(this, &vr_scanning::change_filled));
		connect_copy(add_button("is_filled_2", "shortcut='o'")->click, rebind(this, &vr_scanning::change_filled_2));
		//connect_copy(add_button("SICP")->click, rebind(this, &vr_rgbd::on_reg_SICP_cb));

		add_member_control(this, "rgbd_controller_index", rgbd_controller_index, "value_slider",
						   "min=0;max=3;ticks=true");

		add_member_control(this, "ray_length", ray_length, "value_slider", "min=0.1;max=10;log=true;ticks=true");
		bool show = begin_tree_node("points", show_points, true, "w=100;align=' '");
		add_member_control(this, "show", show_points, "toggle", "w=50");
		if (show) {
			align("\a");
			add_member_control(this, "max_nr_shown_recorded_pcs", max_nr_shown_recorded_pcs, "value_slider",
							   "min=0;max=100;log=true;ticks=true");
			// add_member_control(this, "sort_points", sort_points, "check");
			if (begin_tree_node("point style", point_style)) {
				align("\a");
				add_gui("point_style", point_style);
				align("\b");
				end_tree_node(point_style);
			}
			align("\b");
			end_tree_node(show_points);
		}

		add_decorator("SICP", "heading", "level=2");
		add_member_control(this, "Max SICP runs", sicp.parameters.max_runs, "value_slider",
						   "min=1;max=500;log=false;ticks=true");
		add_member_control(this, "outer loop", sicp.parameters.max_outer_loop, "value_slider",
						   "min=1;max=100;log=false;ticks=true");
		add_member_control(this, "inner loop", sicp.parameters.max_inner_loop, "value_slider",
						   "min=1;max=20;log=false;ticks=true");
		add_member_control(this, "mu", sicp.parameters.mu, "value_slider", "min=1;max=20;log=false;ticks=true");
		add_member_control(this, "max mu", sicp.parameters.max_mu, "value_slider",
						   "min=1;max=100000;log=false;ticks=true");
		add_member_control(this, "use penalty", sicp.parameters.use_penalty, "toggle");
		add_member_control(this, "p", sicp.parameters.p, "value_slider", "min=0.1;max=1.0;log=false;ticks=true");
		add_member_control(this, "alpha", sicp.parameters.alpha, "value_slider",
						   "min=1.05;max=2.0;log=false;ticks=true");
		add_member_control(this, "stop", sicp.parameters.stop, "value_slider",
						   "min=0.00000001;max=0.001;log=true;ticks=false");
		add_member_control(this, "Computaion Mode", (cgv::type::DummyEnum&)sicp_computation_mode, "dropdown",
						   "enums='DEFAULT,POINT_TO_POINT,POINT_TO_PLANE'");

		if (begin_tree_node("box style", style)) {
			align("\a");
			add_gui("box style", style);
			align("\b");
			end_tree_node(style);
		}
		if (begin_tree_node("movable box style", movable_style)) {
			align("\a");
			add_gui("movable box style", movable_style);
			align("\b");
			end_tree_node(movable_style);
		}
		if (begin_tree_node("intersections", srs)) {
			align("\a");
			add_gui("sphere style", srs);
			align("\b");
			end_tree_node(srs);
		}
}
bool vr_scanning::self_reflect(cgv::reflect::reflection_handler& rh)
{
		return rh.reflect_member("rgbd_controller_index", rgbd_controller_index) &&
			   rh.reflect_member("zoom_in", zoom_in) && rh.reflect_member("zoom_out", zoom_out) &&
			   rh.reflect_member("save_pc", save_pointcloud) &&
			   rh.reflect_member("register_pc", registration_started) &&
			   rh.reflect_member("recording_fps", recording_fps) && 
			   rh.reflect_member("ray_length", ray_length) &&
			   rh.reflect_member("record_frame", record_frame) &&
			   rh.reflect_member("record_all_frames", record_all_frames) &&
			   rh.reflect_member("clear_all_frames", clear_all_frames) &&
			   rh.reflect_member("rgbd_started", rgbd_started) &&
			   rh.reflect_member("rgbd_protocol_path", rgbd_protocol_path);
}
void vr_scanning::on_set(void* member_ptr)
{
		if (member_ptr == &rgbd_started && rgbd_started != rgbd_inp.is_started()) {
			if (rgbd_started)
				start_rgbd();
			else
				stop_rgbd();
		}
		if (member_ptr == &rgbd_protocol_path) {
			rgbd_inp.stop();
			rgbd_inp.detach();
			rgbd_inp.attach_path(rgbd_protocol_path);
			if (rgbd_started)
				start_rgbd();
		}
		update_member(member_ptr);
		post_redraw();
}
void vr_scanning::stream_help(std::ostream& os)
{
	os << "vr_rgbd: no shortcuts defined" << std::endl;
}
bool vr_scanning::handle(cgv::gui::event& e)
{
		// check if vr event flag is not set and don't process events in this case
		if ((e.get_flags() & cgv::gui::EF_VR) == 0)
			return false;
		// check event id
		switch (e.get_kind()) {
		case cgv::gui::EID_KEY: {
			cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
			int ci = vrke.get_controller_index();
			if (ci == rgbd_controller_index && vrke.get_key() == vr::VR_DPAD_DOWN) {
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:
					//rgbd_2_controller_orientation_start_calib = controller_orientation; // V^0 = V
					//rgbd_2_controller_position_start_calib = controller_position;		// r^0 = r
					//in_calibration = true;
					//update_member(&in_calibration);
					break;
				case cgv::gui::KA_RELEASE:
					/*rgbd_2_controller_orientation = transpose(rgbd_2_controller_orientation_start_calib) *
													controller_orientation * rgbd_2_controller_orientation;
					rgbd_2_controller_position =
						  transpose(rgbd_2_controller_orientation_start_calib) *
						  ((controller_orientation * rgbd_2_controller_position + controller_position) -
						   rgbd_2_controller_position_start_calib);
					in_calibration = false;
					update_member(&in_calibration);*/
					break;
				}
			}
			if (ci == 0 && vrke.get_key() == vr::VR_DPAD_LEFT) {
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:
					/*zoom_in = true;
					update_member(&zoom_in);*/
					break;
				case cgv::gui::KA_RELEASE:
					/*zoom_in = false;
					update_member(&zoom_in);*/
					break;
				}
			}
			if (ci == 0 && vrke.get_key() == vr::VR_DPAD_RIGHT) {
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:
					/*zoom_out = true;
					update_member(&zoom_out);*/
					break;
				case cgv::gui::KA_RELEASE:
					/*zoom_out = false;
					update_member(&zoom_out);*/
					break;
				}
			}
			if (ci == rgbd_controller_index && vrke.get_key() == vr::VR_MENU) {
				switch (vrke.get_action()) {
				case cgv::gui::KA_PRESS:
					/*clear_all_frames = true;
					update_member(&clear_all_frames);*/
					break;
				case cgv::gui::KA_RELEASE:
					/*clear_all_frames = false;
					update_member(&clear_all_frames);*/
					break;
				}
			}
			if (ci == 0 && vrke.get_key() == vr::VR_GRIP) {
				return true;
			}
			return true;
		}
		case cgv::gui::EID_THROTTLE: {
			cgv::gui::vr_throttle_event& vrte = static_cast<cgv::gui::vr_throttle_event&>(e);
			break;
		}
		case cgv::gui::EID_STICK: {
			cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
			int ci = vrse.get_controller_index();
			if (ci == rgbd_controller_index && vrse.get_action() == cgv::gui::SA_TOUCH) {
				/*trigger_is_pressed = true;
				cam_pose_points.push_back(controller_position);
				cam_pose_ori.push_back(controller_orientation);
				cam_cord_x.push_back(cam_pose_ori.back().col(0));
				cam_cord_y.push_back(cam_pose_ori.back().col(1));
				cam_cord_z.push_back(cam_pose_ori.back().col(2));
				cam_pose_colors.push_back(rgb(1.0, 0.0, 0.0));
				cam_cord_x_clr.push_back(rgb(1.0, 0.0, 0.0));
				cam_cord_y_clr.push_back(rgb(0.0, 1.0, 0.0));
				cam_cord_z_clr.push_back(rgb(0.0, 0.0, 1.0));
				update_member(&trigger_is_pressed);*/
				break;
			}
			else {
				/*trigger_is_pressed = false;
				update_member(&trigger_is_pressed);*/
				break;
			}
		}							  
		case cgv::gui::EID_POSE:
			cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
			// check for controller pose events
			int ci = vrpe.get_trackable_index();
			if (ci == rgbd_controller_index) {
				controller_orientation = vrpe.get_orientation();
				controller_position = vrpe.get_position();
			}
			if (ci != -1) {
				if (state[ci] == IS_GRAB) {
					//// in grab mode apply relative transformation to grabbed boxes

					//// get previous and current controller position
					//vec3 last_pos = vrpe.get_last_position();
					//vec3 pos = vrpe.get_position();
					//// get rotation from previous to current orientation
					//// this is the current orientation matrix times the
					//// inverse (or transpose) of last orientation matrix:
					//// vrpe.get_orientation()*transpose(vrpe.get_last_orientation())
					//mat3 rotation = vrpe.get_rotation_matrix();
					//// iterate intersection points of current controller
					//for (size_t i = 0; i < intersection_points.size(); ++i) {
					//	if (intersection_controller_indices[i] != ci)
					//		continue;
					//	// extract box index
					//	unsigned bi = intersection_box_indices[i];
					//	// update translation with position change and rotation
					//	movable_box_translations[bi] = rotation * (movable_box_translations[bi] - last_pos) + pos;
					//	// update orientation with rotation, note that quaternions
					//	// need to be multiplied in oposite order. In case of matrices
					//	// one would write box_orientation_matrix *= rotation
					//	movable_box_rotations[bi] = quat(rotation) * movable_box_rotations[bi];
					//	// update intersection points
					//	intersection_points[i] = rotation * (intersection_points[i] - last_pos) + pos;
					//}
				}
				else { // not grab
					//// clear intersections of current controller
					//size_t i = 0;
					//while (i < intersection_points.size()) {
					//	if (intersection_controller_indices[i] == ci) {
					//		intersection_points.erase(intersection_points.begin() + i);
					//		intersection_colors.erase(intersection_colors.begin() + i);
					//		intersection_box_indices.erase(intersection_box_indices.begin() + i);
					//		intersection_controller_indices.erase(intersection_controller_indices.begin() + i);
					//	}
					//	else
					//		++i;
					//}

					//// compute intersections
					//vec3 origin, direction;
					//vrpe.get_state().controller[ci].put_ray(&origin(0), &direction(0));
					//compute_intersections(origin, direction, ci, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));

					//// update state based on whether we have found at least
					//// one intersection with controller ray
					//if (intersection_points.size() == i)
					//	state[ci] = InteractionState::IS_NONE;
					//else if (state[ci] == InteractionState::IS_NONE)
					//	state[ci] = IS_OVER;
				}
				post_redraw();
			}
			return true;
		}
		return false;
}
bool vr_scanning::init(cgv::render::context& ctx)
{
		cgv::render::ref_point_renderer(ctx, 1);

		if (!cgv::utils::has_option("NO_OPENVR"))
			ctx.set_gamma(1.0f);
		cgv::gui::connect_vr_server(true);

		auto view_ptr = find_view_as_node();
		if (view_ptr) {
			view_ptr->set_eye_keep_view_angle(dvec3(0, 4, -4));
			// if the view points to a vr_view_interactor
			vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
			if (vr_view_ptr) {
				// configure vr event processing
				vr_view_ptr->set_event_type_flags(cgv::gui::VREventTypeFlags(
					  cgv::gui::VRE_KEY + cgv::gui::VRE_ONE_AXIS + cgv::gui::VRE_ONE_AXIS_GENERATES_KEY +
					  cgv::gui::VRE_TWO_AXES + cgv::gui::VRE_TWO_AXES_GENERATES_DPAD + cgv::gui::VRE_POSE));
				vr_view_ptr->enable_vr_event_debugging(false);
				// configure vr rendering
				vr_view_ptr->draw_action_zone(false);
				vr_view_ptr->draw_vr_kits(true);
				vr_view_ptr->enable_blit_vr_views(true);
				vr_view_ptr->set_blit_vr_view_width(200);
			}
		}
		cgv::render::ref_box_renderer(ctx, 1);
		cgv::render::ref_sphere_renderer(ctx, 1);
		cgv::render::ref_arrow_renderer(ctx, 1);
		return true;
}
void vr_scanning::clear(cgv::render::context& ctx)
{
		cgv::render::ref_point_renderer(ctx, -1);
		cgv::render::ref_box_renderer(ctx, -1);
		cgv::render::ref_sphere_renderer(ctx, -1);
		cgv::render::ref_arrow_renderer(ctx, -1);
}
void vr_scanning::draw_pc(cgv::render::context& ctx, const point_cloud& pc)
{
		if (pc.get_nr_points() == 0)
			return;
		auto& pr = cgv::render::ref_point_renderer(ctx);
		pr.set_position_array(ctx, &pc.pnt(0), pc.get_nr_points(), sizeof(point_cloud::Pnt));
		pr.set_color_array(ctx, &pc.clr(0), pc.get_nr_points(), sizeof(point_cloud::Clr));
		if (pr.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)pc.get_nr_points());
			pr.disable(ctx);
		}
}

void vr_scanning::draw(cgv::render::context& ctx)
{
		if (show_points) {
			auto& pr = cgv::render::ref_point_renderer(ctx);
			pr.set_render_style(point_style);
			pr.set_y_view_angle((float)vr_view_ptr->get_y_view_angle());
			draw_pc(ctx, current_pc);

			size_t begin = 0;
			size_t end = recorded_pcs.size();
			if (end > max_nr_shown_recorded_pcs)
				begin = end - max_nr_shown_recorded_pcs;

			for (size_t i = begin; i < end; ++i)
				draw_pc(ctx, recorded_pcs[i]);
		}
		if (vr_view_ptr) {
			std::vector<vec3> P;
			std::vector<rgb> C;
			const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
			if (state_ptr) {
				for (int ci = 0; ci < 2; ++ci)
					if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
						vec3 ray_origin, ray_direction;
						state_ptr->controller[ci].put_ray(&ray_origin(0), &ray_direction(0));
						P.push_back(ray_origin);
						P.push_back(ray_origin + ray_length * ray_direction);
						rgb c(float(1 - ci), 0.5f * (int)state[ci], float(ci));
						C.push_back(c);
						C.push_back(c);
					}
			}
			if (P.size() > 0) {
				cgv::render::shader_program& prog = ctx.ref_default_shader_program();
				int pi = prog.get_position_index();
				int ci = prog.get_color_index();
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
				cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, C);
				cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
				glLineWidth(3);
				prog.enable(ctx);
				glDrawArrays(GL_LINES, 0, (GLsizei)P.size());
				prog.disable(ctx);
				cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
				cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
				glLineWidth(1);
			}
		}
		// draw static boxes
		cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		renderer.set_render_style(style);
		renderer.set_box_array(ctx, boxes);
		renderer.set_color_array(ctx, box_colors);
		if (renderer.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
		}
		renderer.disable(ctx);

		// draw dynamic boxes
		renderer.set_render_style(movable_style);
		renderer.set_box_array(ctx, movable_boxes);
		renderer.set_color_array(ctx, movable_box_colors);
		renderer.set_translation_array(ctx, movable_box_translations);
		renderer.set_rotation_array(ctx, movable_box_rotations);
		if (renderer.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)movable_boxes.size());
		}
		renderer.disable(ctx);

		// draw intersection points
		if (!intersection_points.empty()) {
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_position_array(ctx, intersection_points);
			sr.set_color_array(ctx, intersection_colors);
			sr.set_render_style(srs);
			if (sr.validate_and_enable(ctx)) {
				glDrawArrays(GL_POINTS, 0, (GLsizei)intersection_points.size());
				sr.disable(ctx);
			}
		}

		// draw pose of camera
		if (!cam_pose_points.empty()) {
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_position_array(ctx, cam_pose_points);
			sr.set_color_array(ctx, cam_pose_colors);
			sr.set_render_style(srs);
			if (sr.validate_and_enable(ctx)) {
				glDrawArrays(GL_POINTS, 0, (GLsizei)cam_pose_colors.size());
				sr.disable(ctx);
			}
		}
		// draw coordinate of camera
		if (!cam_pose_points.empty()) {
			cgv::render::arrow_renderer& a_renderer = cgv::render::ref_arrow_renderer(ctx);
			a_renderer.set_render_style(ars);
			a_renderer.set_position_array(ctx, cam_pose_points);
			a_renderer.set_color_array(ctx, cam_cord_x_clr);
			a_renderer.set_direction_array(ctx, cam_cord_x);
			a_renderer.render(ctx, 0, cam_pose_points.size());
		}
		if (!cam_pose_points.empty()) {
			cgv::render::arrow_renderer& a_renderer = cgv::render::ref_arrow_renderer(ctx);
			a_renderer.set_render_style(ars);
			a_renderer.set_position_array(ctx, cam_pose_points);
			a_renderer.set_color_array(ctx, cam_cord_y_clr);
			a_renderer.set_direction_array(ctx, cam_cord_y);
			a_renderer.render(ctx, 0, cam_pose_points.size());
		}
		if (!cam_pose_points.empty()) {
			cgv::render::arrow_renderer& a_renderer = cgv::render::ref_arrow_renderer(ctx);
			a_renderer.set_render_style(ars);
			a_renderer.set_position_array(ctx, cam_pose_points);
			a_renderer.set_color_array(ctx, cam_cord_z_clr);
			a_renderer.set_direction_array(ctx, cam_cord_z);
			a_renderer.render(ctx, 0, cam_pose_points.size());
		}
}
void vr_scanning::on_reg_SICP_cb()
{
		point_cloud source_pc, target_pc;
		// last frame in the recorded_pcs
		//copy_pointcloud(recorded_pcs.front(), target_pc);
		// current frame
		//copy_pointcloud(intermediate_pc, source_pc);
		if (!target_pc.has_normals()) {
			static constexpr int k = 15;
			ann_tree neighborhood;
			neighborhood.build(target_pc);
			neighbor_graph graph;
			graph.build(target_pc.get_nr_points(), k, neighborhood);

			normal_estimator normi = normal_estimator(target_pc, graph);
			normi.compute_plane_bilateral_weighted_normals(true);
		}
		if (!source_pc.has_normals()) {
			static constexpr int k = 15;
			ann_tree neighborhood;
			neighborhood.build(source_pc);
			neighbor_graph graph;
			graph.build(source_pc.get_nr_points(), k, neighborhood);

			normal_estimator normi = normal_estimator(source_pc, graph);
			normi.compute_plane_bilateral_weighted_normals(true);
		}
		if (source_pc.has_normals() && target_pc.has_normals()) {
			std::cout << "both of them have normals" << std::endl;
		}
		sicp.set_source_cloud(source_pc);
		sicp.set_target_cloud(target_pc);
		vec3 translation, offset;
		mat3 rotation;
		sicp.parameters.max_runs = 20;
		sicp.parameters.p = 0.4f;
		//sicp.register_point_cloud(cgv::pointcloud::SICP::CM_POINT_TO_POINT, rotation, translation);
		sicp.register_point_cloud(cgv::pointcloud::SICP::CM_POINT_TO_PLANE, rotation, translation);
		std::cout << "SICP rot:\n " << rotation << "SICP t:\n" << translation << '\n';
		vec3 mean = std::accumulate(&source_pc.pnt(0), &source_pc.pnt(0) + source_pc.get_nr_points(), vec3(0, 0, 0)) /
					((float)source_pc.get_nr_points());
		// need to de-mean for rotation
		source_pc.translate(-mean);
		// do rotation
		source_pc.rotate(cgv::math::quaternion<float>(rotation));
		// do translation and reapply mean
		source_pc.translate(translation + mean);

		post_redraw();
}

void vr_scanning::open_image()
{
	/*cv::Mat a;
	a = cv::imread("D://7.png");
	cv::imshow("123", a);*/











	//uint8_t uarr[] = { 1,2,3,4,5,6,7,8,9,10,11,12 };
	//int rows = 2;
	//int cols = 2;
	//cv::Size sz(cols, rows);

	//cv::Mat mat1(sz, CV_8UC3, uarr);
	//cv::Mat mat2(rows, cols, CV_8UC3, uarr);

	//std::cout << "mat1: \n" << mat1 << "\n\nmat2:\n" << mat2 << std::endl;

	//if (record_imgs.size() < 3)
	//	return;

	//cv::Mat img_1 = record_imgs.at(1);
	//cv::Mat img_2 = cv::imread("D://7.png");
	//cv::imshow("img1", img_1);
	//cv::imshow("img2", img_2);

	////-- initialize
	//std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
	//cv::Mat descriptors_1, descriptors_2;
	//cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
	//cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

	//cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

	////-- Oriented FAST
	//detector->detect(img_1, keypoints_1);
	//detector->detect(img_2, keypoints_2);

	////-- BRIEF 
	//descriptor->compute(img_1, keypoints_1, descriptors_1);
	//descriptor->compute(img_2, keypoints_2, descriptors_2);

	//std::cout << "size: " << keypoints_1.size() << "x y: " << keypoints_1.at(2).pt.x << " " << keypoints_1.at(2).pt.y << std::endl;
	//cv::Mat outimg1;
	//drawKeypoints(img_1, keypoints_1, outimg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
	//
	//cv::imshow("ORB", outimg1);

	//// Hamming Distance
	//std::vector<cv::DMatch> matches;
	////BFMatcher matcher ( NORM_HAMMING );
	//matcher->match(descriptors_1, descriptors_2, matches);

	//double min_dist = 10000, max_dist = 0;

	//for (int i = 0; i < descriptors_1.rows; ++i)
	//{
	//	double dist = matches[i].distance;
	//	if (dist < min_dist) min_dist = dist;
	//	if (dist > max_dist) max_dist = dist;
	//}

	//std::vector<cv::DMatch> good_matches;
	//for (int i = 0; i < descriptors_1.rows; ++i)
	//{
	//	if (matches[i].distance <= std::max(2 * min_dist, 30.0))
	//	{
	//		good_matches.emplace_back(matches[i]);
	//	}
	//}
	//keypoints_1.at(good_matches[1].queryIdx).pt.x;
	//for (int i = 0; i < good_matches.size(); ++i)
	//{
	//	std::cout << "i: " << i << "x y: " << good_matches[i].queryIdx << " " << good_matches[i].trainIdx << std::endl;
	//}
	//std::cout << "number of matches: " << good_matches.size() << std::endl;
	//cv::Mat img_match;
	//cv::Mat img_goodmatch;
	//drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
	//drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
	//cv::imshow("all matched pairs", img_match);
	//cv::imshow("optimized pairs", img_goodmatch);
	//cv::waitKey(0);


















}

void vr_scanning::change_filled()
{
	std::cout << "is_filled: " << is_filled << " " << is_filled_2 << std::endl;
	if(!is_filled)
		is_filled = true;
}

void vr_scanning::change_filled_2()
{
	std::cout << "is_filled_2: " << is_filled << " " << is_filled_2 << std::endl;
	if (!is_filled_2)
		is_filled_2 = true;
}

#include <cgv/base/register.h>

cgv::base::object_registration<vr_scanning> vr_scanning_reg("");

///@}