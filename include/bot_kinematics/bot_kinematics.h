#ifndef BOT_KINEMATICS_H
#define BOT_KINEMATICS_H

#include <Eigen/Dense>
#include "bot_kinematics/bot_parameters.h"
#include <cmath>

namespace bot_kinematics
{
	/*
	 *A struct to store parameters
	 */
	template <typename T>
	struct Parameters
	{
		static_assert(std::is_floating_point<T>::value,
									"parameters must be templatized with floating point type");

		T a1, a2, a3, l1, l2, l3, t1, t3;//these are arbitrary names which you can provide as you like
	};

	/*
	 *Function to make operator << compatible with datatypes
	 */
	template <typename T>
	std::ostream& operator<<(std::ostream& os, const Parameters<T>& params)
	{
	  os << "Distances: [" << params.a1 << " "
	     << params.a2 << " "
	     << params.a3 << " " << params.l1 << " "
	     << params.l2 << " " << params.l3 << " "
	     << params.t1 << params.t3 << "]\n";
	  return os;
	}

	/**
	* Typedef equivalent to Eigen::Isometry3d for T = double and Eigen::Isometry3f for
	* T = float.
	*/
	template <typename T>
	using Transform = Eigen::Transform<T, 3, Eigen::Isometry>;

	/**
	*to find the ik for a given pose.
	*/
	template <typename T>
	void inverse(const Parameters<T>& p, const Transform<T>& pose, T* out) noexcept;

	/**
	*to find the fk for a given joint angles.
	*/
	template <typename T>
	Transform<T> forward(const Parameters<T>& p, const T* qs) noexcept;

	template <typename T>
	void inverse(const Parameters<T>& p, const Transform<T>& pose, T* out) noexcept
	{
		using Mat = Eigen::Matrix<T, 4, 4>;

		const auto& matrix = pose.matrix();

		T a1=p.a1,a2=p.a2,a3=p.a3,l1=p.l1,l2=p.l2,l3=p.l3,t1=p.t1,t3=p.t3;

		T X=matrix(0,3);
		T Y=matrix(1,3);
		T Z=matrix(2,3);
		/*
		 *Write your ik solution here the x,y,z coordinates are given above.
		 *the variable 'matrix' is 4x4 transformation matrix for position and orientation in base(world) frame.
		 */

		//My code for checking IK (only position is checked, use 'forward' function to check orientation also)
		int number_of_joint_angles=2;//change the number based on your bot
		double error_margin=pow(10,-2);// set allowed error margin
		bool flag=false;
		std::cout<<"error margin: "<<error_margin<<"\n";// comment if you dont want to print

		// the sines and cosines for your matrix, this example only uses only 2 joint angles and more if needed
		T c1=cos(theta1);
		T s1=sin(theta1);
		T c2=cos(theta2);
		T s2=sin(theta2);

		Mat t;
		t<< c1*c2,-c1*s2, s1,a3*c1*c2,
			  s1*c2,-s1*s2,-c1,a3*s1*c2,
			     s2,    c2,  0,a3*s2+a2,
			      0,     0,  0,       1;


		flag=false;

		for(int i=0;i<number_of_joint_angles;i++)
		{
			std::cout<<" error : "<<(matrix(i,3)-t(i,3))<<" ";
			if(error_margin<fabs(matrix(i,3)-t(i,3)))
			{
				flag=false;
				break;
			}
			else
				flag =true;
		}

		if(flag)
		{
			//the solutions are added here as out[(index of joint angle)+(how many'th solution)]=theta_i_j
			out[0]=theta1;
			out[1]=theta2;
			//if you multiple solution just follow the above instruction(sorry for the awkward english!).
		}
		}

	template <typename T>
	Transform<T> forward(const Parameters<T>& p, const T* qs) noexcept
	{
		using Matrix = Eigen::Matrix<T, 4, 4>;
		using Vector = Eigen::Matrix<T, 3, 1>;
		using Rotation=const Eigen::Matrix<T, 3, 3>;

		//the below declarations depends entirely on the number of joint angles you have , in this case it is 3
		T q[3];
		q[0] = qs[0];
		q[1] = qs[1];
		q[2] = qs[2];

		T s1 = std::sin(q[0]);
		T s2 = std::sin(q[1]);
		T s3 = std::sin(q[2]);


		T c1 = std::cos(q[0]);
		T c2 = std::cos(q[1]);
		T c3 = std::cos(q[2]);

		//arbitrary transformation matrices based on DH rule.
		Matrix t01;
		t01 << c1,s1, 0, 0,
			 s1, c1, 0, 0,
			 0 , 0 , 1, p.l1+p.t1,
			 0 , 0 , 1, 1;

		Matrix t12;
		t12 << s2,c2, 0, p.a1,
			   0,  0,1,p.l2+p.l3,
			  c2,s2, 0,  0,
			   0,  0, 0,  1;

		Matrix t23;
		t23 << c3,s3, 0, 0,
		 	  0,  0,1,p.l3+p.a2+p.t1,
		 	 s3, c3, 0, 0,
		 	  0,  0, 0, 1;

			Matrix t34;
			t34 << 1,0,0,p.t3,
				 0,1,0, 0,
				 0,0,1,p.a3,
				 0,0,0, 1;

		//add more matrices if needed

		 Matrix t04=t01*t12*t23*t34;

		Transform<T> i;
		i.matrix()=t04;//convert the simple 4x4 matrix to a transformation matrix (solve data type issues)

		return i;
	}


} // end namespace bot_kinematics

#endif // BOT_KINEMATICS_H
