#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "igl/dqs.h"
#include "Eigen/dense"
#include <functional>

int k, o = 0;
bool yossi = false;

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
	parents.resize(Num_Of_Joints+1);
	scale = 1;
	//Initialize vT, vQ
	vT.resize(17);
	vQ.resize(17);
	
	

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
			//Create_bounding_box();
			if (selected_data_index == 0)
				V = data().V;
			
			
		}
		nameFileout.close();
	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);
	
	//Create_Linkaixs(0);
	
	//Find points for skelton
	
		double z = -0.8*scale;
		for (int i = 0; i < skelton.size(); i++)
		{	
			skelton.at(i) = Eigen::Vector3d(0, 0, z);
			z = z + 0.1*scale;
			
		}
	
	//Draw the skelton points
	Eigen::MatrixXd V_box(17, 3);
	V_box <<
		0, 0, -0.8,
		0, 0, -0.7,
		0, 0, -0.6,
		0, 0, -0.5,
		0, 0, -0.4,
		0, 0, -0.3,
		0, 0, -0.2,
		0, 0, -0.1,
		0, 0, 0,
		0, 0, 0.1,
		0, 0, 0.2,
		0, 0, 0.3,
		0, 0, 0.4,
		0, 0, 0.5,
		0, 0, 0.6,
		0, 0, 0.7,
		0, 0, 0.8,
		// Corners of the bounding box
		data().add_points(V_box, Eigen::RowVector3d(1, 0, 1));
	
	

	//Calaulate the weights for each vertex
	Calculate_Weights();
	data().MyRotate(Eigen::Vector3d(0, 1, 0), 3.14 / 2);
	
	//Create Joints
	//the first joint that dont have a parent
	Joints.emplace_back();
	Joints.at(0).MyTranslate(skelton.at(0), true);
	//Joints.at(0).SetCenterOfRotation(center.transpose());
	parents[0] = -1;
	//std::cout << parents[0] << "\n";
	//the 16 other joint that have parents
	for (int i = 0; i < Num_Of_Joints; i++)
	{
		parents[i + 1] = i;
		Joints.emplace_back();
		Joints.at(i + 1).MyTranslate(skelton.at(i + 1), true);
		//Joints.at(i).SetCenterOfRotation(center.transpose());
		//std::cout << parents[i + 1] <<"\n";
	}
	
	
	destination_position = skelton[Num_Of_Joints];
	U = V;
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
	std::cout << "---------------------------------end of initialization----------------------------------- \n";
	//Set_Tip();
}

SandBox::~SandBox()
{

}

void SandBox::Animate()
{
	
	if (isActive)
	{
		if (left)
		{
			destination_position = Eigen::Vector3d(0, 0, -0.03);


		}
		if (right)
		{

			destination_position = Eigen::Vector3d(0, 0, 0.03);

		}
		if (up)
		{

			destination_position = Eigen::Vector3d(0, 0.03, 0);

		}
		if (down)
		{


			destination_position = Eigen::Vector3d(0, -0.03, 0);
		}


		//Move The Snake
		CalcNextPosition();
		igl::dqs(V, W, vQ, vT, U);
		data_list.at(0).set_vertices(U);
		for (size_t i = 0; i <Num_Of_Joints+1; i++)
		{
			skelton[i] = vT[i];
		}
		
	


	}
	
}


