#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "igl/dqs.h"
#include "Eigen/dense"
#include <functional>
#include <windows.h>
#include <mmsystem.h>
#include <igl/boundary_loop.h>
#include <igl/map_vertices_to_circle.h>
#include <igl/harmonic.h>
//#pragma comment(lib,"Winmm.lib")
//#include <iostream>

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
	rotDir = false;
	snakeEye = 0;
	level = 1;
	score = 0;
	finishLevel = false;
	levelWindow = false;
	skinning = false;

	Num_Of_Joints = 16;
	skelton.resize(Num_Of_Joints+1);
	chain.resize(Num_Of_Joints + 1);
	parentsJoints.resize(Num_Of_Joints+1);
	scale = 1;
	
	//Initialize vT, vQ
	vT.resize(17);
	vQ.resize(17);
	
	//Background Sound
	PlaySound(TEXT("127-bpm-hip-hop-beat-loop.wav"), NULL, SND_LOOP |SND_ASYNC);
	

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
			
			//Create_bounding_box(selected_data_index);
			if (selected_data_index == 1) {
				data().SetCenterOfRotation(center.transpose());
				V = data().V;
				
			}
			if (selected_data_index == 0) {
				data().SetCenterOfRotation(center.transpose());
				
			}
			Initialize_Tree(selected_data_index);
			
			
		}
		nameFileout.close();
	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);
	
	data_list.at(1).MyTranslate(Eigen::Vector3d(0, 0, 0), true);
	data_list.at(0).MyTranslate(Eigen::Vector3d(5, 0, 0), true);
	
	
	//Find points for skelton
	
		double z = -0.8*scale;
		for (int i = 0; i < skelton.size(); i++)
		{	
			skelton.at(i) = Eigen::Vector3d(0, 0, z);
			z = z + 0.1*scale;
			
		}


	
	

	//Calaulate the weights for each vertex
	Calculate_Weights();
	data_list.at(1).MyRotate(Eigen::Vector3d(0, 1, 0), 3.14 / 2);
	
	//Create Joints
	//the first joint that dont have a parent
	Joints.emplace_back();
	Joints.at(0).MyTranslate(skelton.at(0), true);
	
	parentsJoints[0] = -1;
	//the 16 other joint that have parents
	for (int i = 0; i < Num_Of_Joints; i++)
	{
		parentsJoints[i + 1] = i;
		Joints.emplace_back();
		Joints.at(i + 1).MyTranslate(skelton.at(i + 1), true);
		
		
	}
	
	
	U = V;
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
	
}

SandBox::~SandBox()
{

}

void SandBox::SetTexture(int index, std::string texturePath)
{
	Eigen::MatrixXd& V_uv = data_list[index].V_uv;
	Eigen::MatrixXd& V = data_list[index].V;
	Eigen::MatrixXi& F = data_list[index].F;

	Eigen::Vector3d lastRow(F.row(F.rows() / 10 - 1)(0), F.row(F.rows() / 10 - 1)(1), F.row(F.rows() / 10 - 1)(2));
	F.row(F.rows() / 10 - 1)(0) = F.row(F.rows() / 10 - 1)(0);
	F.row(F.rows() / 10 - 1)(1) = F.row(F.rows() / 10 - 1)(0);
	F.row(F.rows() / 10 - 1)(2) = F.row(F.rows() / 10 - 1)(0);

	Eigen::VectorXi bound;
	igl::boundary_loop(F, bound);

	Eigen::MatrixXd bound_uv;
	igl::map_vertices_to_circle(V, bound, bound_uv);

	igl::harmonic(V, F, bound, bound_uv, 1, V_uv);

	V_uv *= 5;
	data_list[index].image_texture(texturePath);
	F.row(F.rows() / 10 - 1)(0) = lastRow(0);
	F.row(F.rows() / 10 - 1)(1) = lastRow(1);
	F.row(F.rows() / 10 - 1)(2) = lastRow(2);

}


void SandBox::Animate()
{
	//Eigen::Vector3d dirVec;
	if (isActive)
	{
		
		if (skinning){
			// Option 1 - without fabrik
			if (left)
			{
				destination_position = Eigen::Vector3d(0, 0, -0.05);


			}
			if (right)
			{

				destination_position = Eigen::Vector3d(0, 0, 0.05);

			}
			if (up)
			{

				destination_position = Eigen::Vector3d(0, 0.05, 0);

			}
			if (down)
			{


				destination_position = Eigen::Vector3d(0, -0.05, 0);
			}


			//Move The Snake
			CalcNextPosition();
			igl::dqs(V, W, vQ, vT, U);
			data_list.at(1).set_vertices(U);
			for (size_t i = 0; i <Num_Of_Joints+1; i++)
			{
				skelton[i] = vT[i];
			}
			
		}
	

	/*
	//Option 2 - without fabrik
	if (skinning){
		
		if (left)
		{
			Joints[Num_Of_Joints].TranslateInSystem(Joints[Num_Of_Joints].MakeTransd(),Eigen::Vector3d(0, 0, -0.05),true);


		}
		if (right)
		{

			Joints[Num_Of_Joints].TranslateInSystem(Joints[Num_Of_Joints].MakeTransd(),Eigen::Vector3d(0, 0, 0.05),true);

		}
		if (down)
		{

			Joints[Num_Of_Joints].TranslateInSystem(Joints[Num_Of_Joints].MakeTransd(),Eigen::Vector3d(0, -0.05, 0),true);

		}
		if (up)
		{

			Joints[Num_Of_Joints].TranslateInSystem(Joints[Num_Of_Joints].MakeTransd(),Eigen::Vector3d(0, 0.05, 0),true);
		}

		//skinning
		for (int i = 0; i < Num_Of_Joints ; i++)
		{
			vT[i] = skelton[i];
		}
		vT[Num_Of_Joints] = (Joints[Num_Of_Joints].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3);
		std::cout << (Joints[Num_Of_Joints].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3) << "\n";
		for (int i = 0; i < Num_Of_Joints; i++)
		{
			vT[i] = vT[i] + (vT[i + 1] - vT[i])*0.1;
		}
		for (size_t i = 0; i < vQ.size(); i++)
		{
			vQ[i] = Eigen::Quaterniond::FromTwoVectors(skelton[i], vT[i]);
		}

		igl::dqs(V, W, vQ, vT, U);
		data_list.at(1).set_vertices(U);
		for (int i = 0; i < Num_Of_Joints + 1; i++)
		{
			skelton[i] = vT[i];
		}
		for (int i = 0; i < Num_Of_Joints; i++)
		{
			Joints[i].MyTranslate(skelton[i],true);

		}
		if (Check_Collision()) {
			left = false;
			right = false;
			up = false;
			down = false;
			//PlaySound(TEXT("mixkit-bonus-earned-in-video-game-2058.wav"), NULL, SND_NOSTOP);
			isActive = !isActive;
		
		}
	}
	
	*/
		
	//Option 4 - Only Collision Detection
	else {
		if (left)
			data().MyTranslate(Eigen::Vector3d(-0.03, 0, 0), true);
		if (right)
			data().MyTranslate(Eigen::Vector3d(0.03, 0, 0), true);
		if (up)
			data().MyTranslate(Eigen::Vector3d(0, 0.03, 0), true);
		if (down)
			data().MyTranslate(Eigen::Vector3d(0, -0.03, 0), true);

		if (Check_Collision())
		{
				score += 10;
				if (score == 50) {
					finishLevel = true;
					left = false;
					right = false;
					up = false;
					down = false;
					isActive = !isActive;
				}
				int y = rand() % 60 - 30;
				double dy = y / 10;
				int x = rand() % 60 - 30;
				double dx = x / 10;
				data_list[0].MyTranslate(Eigen::Vector3d(dx, dy, 0), true);
				
			}
		}
		
		
		
		


	}
}


