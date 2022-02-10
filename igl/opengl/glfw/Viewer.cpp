// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl
{
namespace opengl
{
namespace glfw
{

  void Viewer::Init(const std::string config)
  {
	  

  }

  //------------------------------------------------Project-----------------------------------


  //Collision Detection 
  void Viewer::Create_Linkaixs(int index)
  {
      data_list[index].tree->m_box.center();
      Eigen::MatrixXd V_box(6, 3);
      V_box <<
          0.8, 0, 0.8,
          -0.8, 0, 0.8,
          0, 0.8, 0.8,
          0, -0.8, 0.8,
          0, 0, 0.8,
          0, 0, 2.4;

      data_list[index].add_points(V_box, Eigen::RowVector3d(0, 0, 1));

      Eigen::MatrixXi E_box(3, 2);
      E_box <<
          0, 1,
          2, 3,
          4, 5;

      int x = 1;
      int y = 0;
      int z = 0;
      int count = 0;
      for (unsigned i = 0; i < E_box.rows(); i++) {
          data_list[index].add_edges
          (
              V_box.row(E_box(i, 0)),
              V_box.row(E_box(i, 1)),
              Eigen::RowVector3d(x, y, z)
          );
          z = y;
          y = x;
          x = 0;

      }

  }


  void Viewer::Initialize_Tree(int index) {
      data_list.at(index).tree = new igl::AABB<Eigen::MatrixXd, 3>();
      data_list.at(index).tree->init(data_list.at(index).V, data_list.at(index).F);
    }

  

  void Viewer::Create_bounding_box(int index)
  {
            
          Eigen::AlignedBox <double, 3> box = data_list.at(index).tree->m_box;

          Eigen::MatrixXd V_box(8, 3);
          V_box <<
              box.corner(box.BottomLeftFloor)[0],
              box.corner(box.BottomLeftFloor)[1],
              box.corner(box.BottomLeftFloor)[2],
              box.corner(box.BottomRightFloor)[0],
              box.corner(box.BottomRightFloor)[1],
              box.corner(box.BottomRightFloor)[2],
              box.corner(box.TopLeftFloor)[0],
              box.corner(box.TopLeftFloor)[1],
              box.corner(box.TopLeftFloor)[2],
              box.corner(box.TopRightFloor)[0],
              box.corner(box.TopRightFloor)[1],
              box.corner(box.TopRightFloor)[2],
              box.corner(box.BottomLeftCeil)[0],
              box.corner(box.BottomLeftCeil)[1],
              box.corner(box.BottomLeftCeil)[2],
              box.corner(box.BottomRightCeil)[0],
              box.corner(box.BottomRightCeil)[1],
              box.corner(box.BottomRightCeil)[2],
              box.corner(box.TopLeftCeil)[0],
              box.corner(box.TopLeftCeil)[1],
              box.corner(box.TopLeftCeil)[2],
              box.corner(box.TopRightCeil)[0],
              box.corner(box.TopRightCeil)[1],
              box.corner(box.TopRightCeil)[2];

          // Edges of the bounding box
          Eigen::MatrixXi E_box(12, 2);
          E_box <<
              0, 1,
              1, 3,
              2, 3,
              2, 0,
              4, 5,
              5, 7,
              6, 7,
              6, 4,
              0, 4,
              1, 5,
              2, 6,
              7, 3;

         

          // Plot the edges of the bounding box
          for (unsigned i = 0; i < E_box.rows(); ++i)
              data().add_edges
              (
                  V_box.row(E_box(i, 0)),
                  V_box.row(E_box(i, 1)),
                  Eigen::RowVector3d(1, 0, 0)
              );
              
         
       
      
      
  }

  void Viewer::create_bounding_box(Eigen::AlignedBox <double, 3>& box, int index)
  {
      // Corners of the bounding box
      Eigen::MatrixXd V_box(8, 3);
      V_box <<
          box.corner(box.BottomLeftFloor)[0],
          box.corner(box.BottomLeftFloor)[1],
          box.corner(box.BottomLeftFloor)[2],
          box.corner(box.BottomRightFloor)[0],
          box.corner(box.BottomRightFloor)[1],
          box.corner(box.BottomRightFloor)[2],
          box.corner(box.TopLeftFloor)[0],
          box.corner(box.TopLeftFloor)[1],
          box.corner(box.TopLeftFloor)[2],
          box.corner(box.TopRightFloor)[0],
          box.corner(box.TopRightFloor)[1],
          box.corner(box.TopRightFloor)[2],
          box.corner(box.BottomLeftCeil)[0],
          box.corner(box.BottomLeftCeil)[1],
          box.corner(box.BottomLeftCeil)[2],
          box.corner(box.BottomRightCeil)[0],
          box.corner(box.BottomRightCeil)[1],
          box.corner(box.BottomRightCeil)[2],
          box.corner(box.TopLeftCeil)[0],
          box.corner(box.TopLeftCeil)[1],
          box.corner(box.TopLeftCeil)[2],
          box.corner(box.TopRightCeil)[0],
          box.corner(box.TopRightCeil)[1],
          box.corner(box.TopRightCeil)[2];

      // Edges of the bounding box
      Eigen::MatrixXi E_box(12, 2);
      E_box <<
          0, 1,
          1, 3,
          2, 3,
          2, 0,
          4, 5,
          5, 7,
          6, 7,
          6, 4,
          0, 4,
          1, 5,
          2, 6,
          7, 3;


      // Plot the edges of the bounding box
      for (unsigned i = 0; i < E_box.rows(); ++i)
          data_list.at(index).add_edges
          (
              V_box.row(E_box(i, 0)),
              V_box.row(E_box(i, 1)),
              Eigen::RowVector3d(1, 0, 0)
          );
  }

  bool Viewer::Check_Collision()
  {
      for (int i = 0; i < data_list.size(); i++)
      {
          if (i != selected_data_index)
          {
              //boxesCreated = false;
              if (Check_Collision(*data_list.at(selected_data_index).tree, selected_data_index, *data_list.at(i).tree, i))
                  return true;



          }
      }
      return false;
  }

  bool Viewer::Check_Collision(igl::AABB<Eigen::MatrixXd, 3>& A, int indexA, igl::AABB<Eigen::MatrixXd, 3>& B, int indexB)
  {
      Eigen::AlignedBox <double, 3> A_box = A.m_box;
      Eigen::AlignedBox <double, 3> B_box = B.m_box;



      if (Is_Collide(A_box, indexA, B_box, indexB))
      {
          if (A.is_leaf() & B.is_leaf())
          {
              create_bounding_box(B_box, indexB);
              create_bounding_box(A_box, indexA);
              return true;
          }
          if (A.is_leaf())
          {
              return Check_Collision(A, indexA, *B.m_left, indexB) || Check_Collision(A, indexA, *B.m_right, indexB);
          }
          if (B.is_leaf())
          {
              return Check_Collision(*A.m_left, indexA, B, indexB) || Check_Collision(*A.m_right, indexA, B, indexB);
          }
          return Check_Collision(*A.m_left, indexA, *B.m_left, indexB) || Check_Collision(*A.m_left, indexA, *B.m_right, indexB) || Check_Collision(*A.m_right, indexA, *B.m_left, indexB) || Check_Collision(*A.m_right, indexA, *B.m_right, indexB);
      }
      return false;


  }

  bool Viewer::Is_Collide(Eigen::AlignedBox <double, 3>& A_box, int indexA, Eigen::AlignedBox <double, 3>& B_box, int indexB)
  {

      Eigen::Vector3d ARightCol(data_list.at(indexA).MakeTransd()(0, 3), data_list.at(indexA).MakeTransd()(1, 3), data_list.at(indexA).MakeTransd()(2, 3));
      Eigen::Vector3d BRightCol(data_list.at(indexB).MakeTransd()(0, 3), data_list.at(indexB).MakeTransd()(1, 3), data_list.at(indexB).MakeTransd()(2, 3));
      Eigen::Vector3d C0 = A_box.center();
      Eigen::Vector3d C1 = B_box.center();

      Eigen::Matrix3d A = data_list.at(indexA).GetRotation();
      Eigen::Matrix3d B = data_list.at(indexB).GetRotation();

      Eigen::Vector3d D = (B * C1 + BRightCol) - (A * C0 + ARightCol);

      Eigen::Vector3d A0(A(0, 0), A(1, 0), A(2, 0));
      Eigen::Vector3d A1(A(0, 1), A(1, 1), A(2, 1));
      Eigen::Vector3d A2(A(0, 2), A(1, 2), A(2, 2));

      Eigen::Vector3d B0(B(0, 0), B(1, 0), B(2, 0));
      Eigen::Vector3d B1(B(0, 1), B(1, 1), B(2, 1));
      Eigen::Vector3d B2(B(0, 2), B(1, 2), B(2, 2));

      double a0 = A_box.sizes()(0) / 2;
      double a1 = A_box.sizes()(1) / 2;
      double a2 = A_box.sizes()(2) / 2;
      double b0 = B_box.sizes()(0) / 2;
      double b1 = B_box.sizes()(1) / 2;
      double b2 = B_box.sizes()(2) / 2;

      Eigen::Matrix3d C = data_list.at(indexA).GetRotation().transpose() * data_list.at(indexB).GetRotation();

      double R0, R1, R;
      //CASE 1
      R0 = a0;
      R1 = b0 * abs(C(0, 0)) + b1 * abs(C(0, 1)) + b2 * abs(C(0, 2));
      R = (A0.transpose() * D).norm();
      if (R > R0 + R1)
          return false;

      //CASE 2
      R0 = a1;
      R1 = b0 * abs(C(1, 0)) + b1 * abs(C(1, 1)) + b2 * abs(C(1, 2));
      R = (A1.transpose() * D).norm();
      if (R > R0 + R1)
          return false;

      //CASE 3
      R0 = a2;
      R1 = b0 * abs(C(2, 0)) + b1 * abs(C(2, 1)) + b2 * abs(C(2, 2));
      R = (A2.transpose() * D).norm();
      if (R > R0 + R1)
          return false;

      //CASE 4
      R0 = a0 * abs(C(0, 0)) + a1 * abs(C(1, 0)) + a2 * abs(C(2, 0));
      R1 = b0;
      R = (B0.transpose() * D).norm();
      if (R > R0 + R1)
          return false;

      //CASE 5
      R0 = a0 * abs(C(0, 1)) + a1 * abs(C(1, 1)) + a2 * abs(C(2, 1));
      R1 = b1;
      R = (B1.transpose() * D).norm();
      if (R > R0 + R1)
          return false;

      //CASE 6
      R0 = a0 * abs(C(0, 2)) + a1 * abs(C(1, 2)) + a2 * abs(C(2, 2));
      R1 = b2;
      R = (B2.transpose() * D).norm();
      if (R > R0 + R1)
          return false;

      //CASE 7
      R0 = a1 * abs(C(2, 0)) + a2 * abs(C(1, 0));
      R1 = b1 * abs(C(0, 2)) + b2 * abs(C(0, 1));
      R = (C(1, 0) * A2.transpose() * D - C(2, 0) * A1.transpose() * D).norm();
      if (R > R0 + R1)
          return false;

      //CASE 8
      R0 = a1 * abs(C(2, 1)) + (a2)*abs(C(1, 1));
      R1 = b0 * abs(C(0, 2)) + b2 * abs(C(0, 0));
      R = (C(1, 1) * A2.transpose() * D - C(2, 1) * A1.transpose() * D).norm();
      if (R > R0 + R1)
          return false;

      //CASE 9
      R0 = a1 * abs(C(2, 2)) + a2 * abs(C(1, 2));
      R1 = b0 * abs(C(0, 1)) + b1 * abs(C(0, 0));
      R = (C(1, 2) * A2.transpose() * D - C(2, 2) * A1.transpose() * D).norm();
      if (R > R0 + R1)
          return false;

      //CASE 10
      R0 = a0 * abs(C(2, 0)) + a2 * abs(C(0, 0));
      R1 = b1 * abs(C(1, 2)) + b2 * abs(C(1, 1));
      R = (C(2, 0) * A0.transpose() * D - C(0, 0) * A2.transpose() * D).norm();
      if (R > R0 + R1)
          return false;

      //CASE 11
      R0 = a0 * abs(C(2, 1)) + a2 * abs(C(0, 1));
      R1 = b0 * abs(C(1, 2)) + b2 * abs(C(1, 0));
      R = (C(2, 1) * A0.transpose() * D - C(0, 1) * A2.transpose() * D).norm();
      if (R > R0 + R1)
          return false;

      //CASE 12
      R0 = a0 * abs(C(2, 2)) + a2 * abs(C(0, 2));
      R1 = b0 * abs(C(1, 1)) + b1 * abs(C(1, 0));
      R = (C(2, 2) * A0.transpose() * D - C(0, 2) * A2.transpose() * D).norm();
      if (R > R0 + R1)
          return false;

      //CASE 13
      R0 = a0 * abs(C(1, 0)) + a1 * abs(C(0, 0));
      R1 = b1 * abs(C(2, 2)) + b2 * abs(C(2, 1));
      R = (C(0, 0) * A1.transpose() * D - C(1, 0) * A0.transpose() * D).norm();
      if (R > R0 + R1)
          return false;

      //CASE 14
      R0 = a0 * abs(C(1, 1)) + a1 * abs(C(0, 1));
      R1 = b0 * abs(C(2, 2)) + b2 * abs(C(2, 0));
      R = (C(0, 1) * A1.transpose() * D - C(1, 1) * A0.transpose() * D).norm();
      if (R > R0 + R1)
          return false;

      //CASE 15
      R0 = a0 * abs(C(1, 2)) + a1 * abs(C(0, 2));
      R1 = b0 * abs(C(2, 1)) + b1 * abs(C(2, 0));
      R = (C(0, 2) * A1.transpose() * D - C(1, 2) * A0.transpose() * D).norm();
      if (R > R0 + R1)
          return false;
      return true;
  }


 

  //Weights Calculation
  Eigen::VectorXd Viewer::creatWiVector(Eigen::Vector4d temp)
  {
      Eigen::VectorXd Wi;
      Wi.resize(17);
      double weight1 = temp[0];
      double index1 = temp[1];
      double weight2 = temp[2];
      double index2 = temp[3];
      /*std::cout << weight1 << " - weight1\n";
      std::cout << index1 << " - index1\n";
      std::cout << weight2 << " - weight2\n";
      std::cout << index2 << " - index2\n";
      */
      for (double i = 0; i < 17; i++)
      {
          if (i == index1)
              Wi[index1] = weight1;
          else {
              if (i == index2)
                  Wi[index2] = weight2;
              else
                  Wi[i] = 0;
          }
      }
      return Wi;

  }

  void Viewer::Calculate_Weights()
  {
      int numOfV = data_list.at(1).V.rows();
      Eigen::MatrixXd V = data_list.at(1).V;
      W.resize(numOfV, 17);
      double zCord;
      double lowBound, upBound;
      double weight1, weight2;
      Eigen::VectorXd weightVector;
      for (int i = 0; i < numOfV; i++)
      {
          zCord = V.row(i)[2];
          lowBound = (floor(zCord * 10)) / 10;
          upBound = (ceil(zCord * 10)) / 10;
          weight1 = abs(zCord - upBound) * 10;
          weight2 = 1 - weight1;
          W.row(i) = creatWiVector(Eigen::Vector4d(weight1, lowBound * 10 + 8, weight2, upBound * 10 + 8));
         // std::cout << i << " weightVector \n";
          //std::cout << W.row(i) << "\n";
      }
     
      
  }



  //Snake Movment

  /*
  void Viewer::Set_Tip()
  {
      Eigen::Vector3d O = (Joints[0].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3);
      Eigen::Matrix3d r;
      Eigen::Vector3d l;
      //start_position = O + data_list[1].GetRotation() * Eigen::Vector3d(0, 0, -0.8);
      //skelton[0] = O + Joints[0].GetRotation() * Eigen::Vector3d(0, 0, -0.8);
      skelton[0] = O + Joints[0].GetRotation() * Eigen::Vector3d(0, 0, -0.8);
      // std::cout << "the current position of the 0 joint is: \n";
       //std::cout << skelton[0] << "\n";
      for (int i = 1; i < Num_Of_Joints + 1; i++)
      {

          r = Joints[i].GetRotation();
          for (int j = i - 1; j > 0; j--)
          {
              r = Joints[j].GetRotation() * r;
          }

          l(0) = 0;
          l(1) = 0;
          l(2) = 0.1 * scale;
          O = O + r * l;

          skelton[i] = O;
          //std::cout << "the current position of the (skelton[i])" << i << " joint is: \n";
          //std::cout << skelton[i] << "\n";
      }

      //tip_position = O;

  }
  */

  void Viewer::CalcNextPosition()
  {
      for (int i = 0; i < Num_Of_Joints + 1; i++)
      {
          vT[i] = skelton[i];
      }
      for (size_t i = 0; i < Num_Of_Joints; i++)
      {
          vT[i] = vT[i] + (vT[i + 1] - vT[i]) / 6;
      }
      vT[Num_Of_Joints] = vT[Num_Of_Joints] + destination_position;
  }


  /*
  void Viewer::Fabrik()
  {
      chain[Num_Of_Joints] = (data_list.at(0).MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3);;
      for (int i = Num_Of_Joints - 1; i >= 0; i--)
      {
          Eigen::Vector3d curr = chain[i];
          Eigen::Vector3d next = chain[i + 1];
          double ri = (curr - next).norm();
          double offset = 1.6 / ri;
          chain[i] = (1 - offset) * next + curr * offset;

      }

  }
  */
  /*
    void Viewer::moveChain()
    {
        Eigen::Vector3d currVec;
        Eigen::Vector3d destVec;
        double angle;
        double tempAngle;
        for (int i = 0; i < Num_Of_Joints; i++)
        {
            currVec = skelton[i] - skelton[i + 1];
            destVec = chain[i] - chain[i + 1];
            tempAngle = currVec.normalized().dot(destVec.normalized());
            if (tempAngle > 1)
                tempAngle = 1;
            if (tempAngle < -1)
                tempAngle = -1;
            angle = acos(tempAngle);
            Eigen::Vector3d almostrotVec = currVec.cross(destVec);
            Eigen::Vector4d rotVec(almostrotVec(0), almostrotVec(1), almostrotVec(2), 0);
            Joints[i + 1].MyRotate(((CalcParentsTransJoints(i + 1) * Joints[i + 1].MakeTransd()).inverse() * rotVec).head(3), angle / 10);
              
        }

    }
    */
  /*
    Eigen::Matrix4d Viewer::CalcParentsTransJoints(int index)
    {
        Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();

        for (int i = index; parents[i] >= 0; i = parents[i])
        {
            //std::cout << "parent matrix:\n" << scn->data_list[scn->parents[i]].MakeTrans() << std::endl;
            prevTrans = Joints[parents[i]].MakeTransd() * prevTrans;
        }

        return prevTrans;
    }
    */
  
  
  //------------------------------------------------Project-----------------------------------



  IGL_INLINE Viewer::Viewer():
    data_list(1),
    selected_data_index(0),
    next_data_id(1),
	isPicked(false),
	isActive(false)
  {
    data_list.front().id = 0;

  

    // Temporary variables initialization
   // down = false;
  //  hack_never_moved = true;
    scroll_position = 0.0f;

    // Per face
    data().set_face_based(false);

    
#ifndef IGL_VIEWER_VIEWER_QUIET
    const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
);
    std::cout<<usage<<std::endl;
#endif
  }

  IGL_INLINE Viewer::~Viewer()
  {
  }

  IGL_INLINE bool Viewer::load_mesh_from_file(
      const std::string & mesh_file_name_string)
  {

    // Create new data slot and set to selected
    if(!(data().F.rows() == 0  && data().V.rows() == 0))
    {
      append_mesh();
    }
    data().clear();

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }

    std::string extension = mesh_file_name_string.substr(last_dot+1);

    if (extension == "off" || extension =="OFF")
    {
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      if (!igl::readOFF(mesh_file_name_string, V, F))
        return false;
      data().set_mesh(V,F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;

      if (!(
            igl::readOBJ(
              mesh_file_name_string,
              V, UV_V, corner_normals, F, UV_F, fNormIndices)))
      {
        return false;
      }

      data().set_mesh(V,F);
      if (UV_V.rows() > 0)
      {
          data().set_uv(UV_V, UV_F);
      }

    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }

    data().compute_normals();
    data().uniform_colors(Eigen::Vector3d(51.0/255.0,43.0/255.0,33.3/255.0),
                   Eigen::Vector3d(255.0/255.0,228.0/255.0,58.0/255.0),
                   Eigen::Vector3d(255.0/255.0,235.0/255.0,80.0/255.0));

    // Alec: why?
    if (data().V_uv.rows() == 0)
    {
      data().grid_texture();
    }
    

    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->post_load())
    //    return true;

    return true;
  }

  IGL_INLINE bool Viewer::save_mesh_to_file(
      const std::string & mesh_file_name_string)
  {
    // first try to load it with a plugin
    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->save(mesh_file_name_string))
    //    return true;

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      // No file type determined
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }
    std::string extension = mesh_file_name_string.substr(last_dot+1);
    if (extension == "off" || extension =="OFF")
    {
      return igl::writeOFF(
        mesh_file_name_string,data().V,data().F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;

      return igl::writeOBJ(mesh_file_name_string,
          data().V,
          data().F,
          corner_normals, fNormIndices, UV_V, UV_F);
    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }
    return true;
  }
 
  IGL_INLINE bool Viewer::load_scene()
  {
    std::string fname = igl::file_dialog_open();
    if(fname.length() == 0)
      return false;
    return load_scene(fname);
  }

  IGL_INLINE bool Viewer::load_scene(std::string fname)
  {
   // igl::deserialize(core(),"Core",fname.c_str());
    igl::deserialize(data(),"Data",fname.c_str());
    return true;
  }

  IGL_INLINE bool Viewer::save_scene()
  {
    std::string fname = igl::file_dialog_save();
    if (fname.length() == 0)
      return false;
    return save_scene(fname);
  }

  IGL_INLINE bool Viewer::save_scene(std::string fname)
  {
    //igl::serialize(core(),"Core",fname.c_str(),true);
    igl::serialize(data(),"Data",fname.c_str());

    return true;
  }

  IGL_INLINE void Viewer::open_dialog_load_mesh()
  {
    std::string fname = igl::file_dialog_open();

    if (fname.length() == 0)
      return;
    
    this->load_mesh_from_file(fname.c_str());
  }

  IGL_INLINE void Viewer::open_dialog_save_mesh()
  {
    std::string fname = igl::file_dialog_save();

    if(fname.length() == 0)
      return;

    this->save_mesh_to_file(fname.c_str());
  }

  IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
  {
    assert(data_list.size() >= 1);

    data_list.emplace_back();
    selected_data_index = data_list.size()-1;
    data_list.back().id = next_data_id++;
    //if (visible)
    //    for (int i = 0; i < core_list.size(); i++)
    //        data_list.back().set_visible(true, core_list[i].id);
    //else
    //    data_list.back().is_visible = 0;
    return data_list.back().id;
  }

  IGL_INLINE bool Viewer::erase_mesh(const size_t index)
  {
    assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
    assert(data_list.size() >= 1);
    if(data_list.size() == 1)
    {
      // Cannot remove last mesh
      return false;
    }
    data_list[index].meshgl.free();
    data_list.erase(data_list.begin() + index);
    if(selected_data_index >= index && selected_data_index > 0)
    {
      selected_data_index--;
    }

    return true;
  }

  IGL_INLINE size_t Viewer::mesh_index(const int id) const {
    for (size_t i = 0; i < data_list.size(); ++i)
    {
      if (data_list[i].id == id)
        return i;
    }
    return 0;
  }

  Eigen::Matrix4d Viewer::CalcParentsTrans(int indx) 
  {
	  Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();

	  for (int i = indx; parents[i] >= 0; i = parents[i])
	  {
		  //std::cout << "parent matrix:\n" << scn->data_list[scn->parents[i]].MakeTrans() << std::endl;
		  prevTrans = data_list[parents[i]].MakeTransd() * prevTrans;
	  }

	  return prevTrans;
  }

} // end namespace
} // end namespace
}
