vr_scanning.cxx 
	line 593 define the resolution of mesh e.g.(...ivec3(200, 200, 200), 0.01) or (...ivec3(100, 100, 100), 0.02)
	line 768 controls if it shows pointcloud
	line 769 controls if it shows mesh

while running program in GUI:
	Press button "load file", select the example file "monitor.txt" to see the result.
	when the camera is started, Press button "save pc" to save the depth and color image info in txt file.
	Press button "save as .obj", the mesh is saved in obj file.(1. In general the obj viewers dont show the vertex color. 
								    2. The normals are calculated in the program but not saved in the obj file)

You can use vr controller to change the pose of camera in the coordinate. But you must make sure it always appears inside the boundingbox.