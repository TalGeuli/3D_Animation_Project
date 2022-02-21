#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>
#include <Eigen/dense>


class Movable
{
public:
	Movable();
	Movable(const Movable& mov);
	Eigen::Matrix4f MakeTransScale();
	Eigen::Matrix4d MakeTransd();
	Eigen::Matrix4d MakeTransScaled();
	Eigen::Matrix3d GetRotation() { return Tout.rotation(); }
	void MyTranslate(Eigen::Vector3d amt, bool preRotation);
	void TranslateInSystem(Eigen::Matrix3d rot, Eigen::Vector3d amt);
	void MyRotate(Eigen::Vector3d rotAxis, double angle);
	void RotateInSystem(Eigen::Vector3d rotAxis, double angle);
	void MyRotate(const Eigen::Matrix3d &rot);
	void MyScale(Eigen::Vector3d amt);
	void SetCenterOfRotation(Eigen::Vector3d amt);
	
	Eigen::Matrix3d GetRotation() const{ return Tout.rotation().matrix(); }

	virtual ~Movable() {}

	//-------------------------------------------------------project---------------------------------------------------------
	void Movable::TranslateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d amt, bool preRotation);
	Eigen::Quaterniond GetRotationQ();
	void Movable::RotateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d rotAxis, double angle);
	Eigen::Vector3d Movable::GetCenter();
	

	//-------------------------------------------------------project----------------------------------------------------------
private:
	Eigen::Affine3d Tout,Tin;
};

