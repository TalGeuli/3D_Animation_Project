#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "igl/dqs.h"
#include "Eigen/dense"
#include <functional>



SandBox::SandBox()
{
	

}

void SandBox::Init(const std::string &config)
{
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	
	right = false;
	left = false;
	up = false;
	down = false;
	
	Num_Of_Joints = 16;
	skelton.resize(Num_Of_Joints+1);
	scale = 1;
	Eigen::RowVector3d center(0, 0, -0.8);

	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file "<<config << std::endl;
	}
	else
	{
		
		while (nameFileout >> item_name)
		{
			std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);
			
			Eigen::RowVector3d center(0, 0, -0.8);
			parents.push_back(-1);
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = false;
			data().point_size = 10;
			data().line_width = 2;
			data().set_visible(false, 1);
			//data().SetCenterOfRotation(Eigen::Vector3d(1, 0, 0));
			//data().MyScale(Eigen::Vector3d(1, 1, scale));
			Create_bounding_box();
			if (selected_data_index == 0)
				V = data().V;
			
			
		}
		nameFileout.close();
	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);
	
	//Create_Linkaixs(0);
	
	double z = -0.8*scale;
	for (int i = 0; i < skelton.size(); i++)
	{
		skelton.at(i) = Eigen::Vector3d(0, 0, z);
		z = z + 0.1*scale;
		//std::cout << skelton.at(i) << "\n";
	}
	
	//Calaulate the weights for each vertex
	Calculate_Weights();
	
	//Create Joints
	Joints.emplace_back();
	for (int i = 1; i <= Num_Of_Joints; i++)
	{
		Joints.emplace_back();
		Joints.at(i).MyTranslate(skelton.at(i), true);
		Joints.at(i).SetCenterOfRotation(center.transpose());
		//vQ.at(i) = Joints[i].GetRotationQ();
	}
	

	



	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));

}

SandBox::~SandBox()
{

}

void SandBox::Animate()
{
	if (isActive)
	{
		if (right)
		{
			Joints[Num_Of_Joints].RotateInSystem(Joints[Num_Of_Joints].MakeTransd(), Eigen::Vector3d(0, 0, 1), 0.05);
			Set_Tip();
		}
		if (left)
		{
			Joints[Num_Of_Joints].RotateInSystem(Joints[Num_Of_Joints].MakeTransd(), Eigen::Vector3d(0, 0, 1), -0.05);
			Set_Tip();
		}
		if (up)
		{
			Joints[Num_Of_Joints].MyRotate(Eigen::Vector3d(1, 0, 0), 0.05);
			Set_Tip();
		}
		if (down)
		{
			Joints[Num_Of_Joints].MyRotate(Eigen::Vector3d(1, 0, 0), -0.05);
			Set_Tip();
		}
		

		Fabrik();
		igl::dqs(V,W,vQ,vT,U);
		data_list[0].set_vertices(U);
	}
}


