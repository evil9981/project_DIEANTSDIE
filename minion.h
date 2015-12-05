#pragma once
#include <chai3d.h>

class minion :
	public chai3d::cShapeSphere
{
public:
	minion(const double& a_radius) : cShapeSphere(a_radius){}
	virtual ~minion();

	double health;
	double speed;

	chai3d::cVector3d calculateForce(const chai3d::cVector3d &globalPosition);
};

