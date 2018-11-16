#include <cstdlib>
#include <cmath>
#include <ctime>
#include "pqp/include/PQP.h"
#include "quaternion.hpp"

quat Quat::random()
{
	//randomly generate a quaternion
	srand(time(0));
	float s=(((double)rand())/RAND_MAX);
	float sigma1=sqrt(1-s);
	float sigma2=sqrt(s);
	float theta1=M_PI*(((double)rand())/RAND_MAX);
	float theta2=M_PI*(((double)rand())/RAND_MAX);
	quat q;
	q.w=cos(theta2)*sigma2;
	q.x=sin(theta1)*sigma1;
	q.y=cos(theta1)*sigma1;
	q.z=sin(theta2)*sigma2;
	return q;
}

float Quat::distance(quat q1, quat q2)
{
	//compute distance between two quaternion
	float lambda=(q1.w*q2.w)+(q1.x*q2.x)+(q1.y*q2.y)+(q1.z*q2.z);
	lambda=lambda*lambda;
	return 1-lambda; //distance in range [0,1] (0=same, 1=180^ apart)
}

quat Quat::slerp(quat q1, quat q2, float param)
{
	//interpolate two quaternions
	//param is the fraction between the two quaternions
	float lambda=(q1.w*q2.w)+(q1.x*q2.x)+(q1.y*q2.y)+(q1.z*q2.z);
	if(lambda<0)
	{
		q2.w=-q2.w;
		q2.x=-q2.x;
		q2.y=-q2.y;
		q2.z=-q2.z;
		lambda=-lambda;
	}
	float r,s;
	if(lambda>0.9995)
	{
		//if they are too close, use linear interpolation
		r=1-param;
		s=param;
	}
	else
	{
		float alpha=acos(lambda);
		float gamma=1/sin(alpha);
		r=sin((1-param)*alpha)*gamma;
		s=sin(param*alpha)*gamma;
	}
	quat newQuat;
	newQuat.w=r*q1.w+s*q2.w;
	newQuat.x=r*q1.x+s*q2.x;
	newQuat.y=r*q1.y+s*q2.y;
	newQuat.z=r*q1.z+s*q2.z;
	float normFactor=sqrt(newQuat.w*newQuat.w+newQuat.x*newQuat.x+newQuat.y*newQuat.y+newQuat.z*newQuat.z);
	newQuat.w=newQuat.w/normFactor;
	newQuat.x=newQuat.x/normFactor;
	newQuat.y=newQuat.y/normFactor;
	newQuat.z=newQuat.z/normFactor;
	return newQuat;

}

void Quat::toMatrix(quat q, PQP_REAL (*R)[3][3])
{
	//set the rotation matrix cooresponding to the given quaternion
	(*R)[0][0]=1-2*q.y*q.y-2*q.z*q.z;
	(*R)[0][1]=2*q.x*q.y-2*q.z*q.w;
	(*R)[0][2]=2*q.x*q.z+2*q.y*q.w;
	(*R)[1][0]=2*q.x*q.y+2*q.z*q.w;
	(*R)[1][1]=1-2*q.x*q.x-2*q.z*q.z;
	(*R)[1][2]=2*q.y*q.z-2*q.x*q.w;
	(*R)[2][0]=2*q.x*q.z-2*q.y*q.w;
	(*R)[2][1]=2*q.y*q.z-2*q.x*q.w;
	(*R)[2][2]=1-2*q.x*q.x-2*q.y*q.y;
}

