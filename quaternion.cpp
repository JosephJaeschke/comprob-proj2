#include <cstdlib>
#include <cmath>
#include "quaternion.h"

#define INT_PARAM 0.5 //interpolation parameter

quat Quat::random()
{
	//randomly generate a quaternion
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

float Quat::dist(quat q1, quat q2)
{
	//compute distance between two quaternion
	float lambda=(q1.w*q2.w)+(q1.x*q2.x)+(q1.y*q2.y)+(q1.z*q2.z);
	lambda=lambda*lambda;
	return 1-lambda; //distance in range [0,1] (0=same, 1=180^ apart)
}

quat Quat::slerp(quat q1, quat q2)
{
	//interpolate two quaternions
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
		r=1-INT_PARAM;
		s=INT_PARAM;
	}
	else
	{
		float alpha=acos(lambda);
		float gamma=1/sin(alpha);
		r=sin((1-INT_PARAM)*alpha)*gamma;
		s=sin(INT_PARAM*alpha)*gamma;
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

float** Quat::toMatrix(quat q)
{
	//return the rotation materix cooresponding to the given quaternion
	float** matrix=new float*[3];
	int i;
	for(i=0;i<3;i++)
	{
		matrix[i]=new float[3];
		int j;
		for(j=0;j<3;j++)
		{
			matrix[i][j]=0; //matrix[height][width]
		}
	}
	matrix[0][0]=1-2*q.y*q.y-2*q.z*q.z;
	matrix[0][1]=2*q.x*q.y-2*q.z*q.w;
	matrix[0][2]=2*q.x*q.z+2*q.y*q.w;
	matrix[1][0]=2*q.x*q.y+2*q.z*q.w;
	matrix[1][1]=1-2*q.x*q.x-2*q.z*q.z;
	matrix[1][2]=2*q.y*q.z-2*q.x*q.w;
	matrix[2][0]=2*q.x*q.z-2*q.y*q.w;
	matrix[2][1]=2*q.y*q.z-2*q.x*q.w;
	matrix[2][2]=1-2*q.x*q.x-2*q.y*q.y;
	return matrix;
}


















