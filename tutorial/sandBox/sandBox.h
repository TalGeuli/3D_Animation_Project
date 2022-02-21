#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"

class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox();
	~SandBox();
	void Init(const std::string& config);
	void SandBox::SetTexture(int index, std::string texturePath);
	double doubleVariable;
private:
	// Prepare array-based edge data structures and priority queue
	
	
	void Animate();
};

