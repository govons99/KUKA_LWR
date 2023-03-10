/* 
LWR_Dynamic_Model_Lib supplies features related to the dynamic model of the KUKA LWR;
in particular, it offers the numerical computation of the mass matrix (7-by-7), the Coriolis vector (7-by-1)
and the gravity vector (7-by-1), beyond the computation of the torques vector (7-by-1).

The dynamic model is B(q)*ddq + c(q,dq) + g(q) = tau
(frictions neglected)

@author Claudio Roberto Gaz <claudio.gaz@diag.uniroma1.it>
@version 1.0
*/

#include <stddef.h>
#include <stdio.h>
//#include <tchar.h>

class CLWR_Dynamic_Model_Lib
{
	// This class is exported from the LWR_Dynamic_Model_Lib.dll
	public:
		// constructor
		CLWR_Dynamic_Model_Lib();

		// get_S returns the numerical computation of the current S(q,dq)
		// inputs:
		//	-	float** S: a pointer to a 7-by-7 matrix, which will be filled with the numerical
		//		values of the matrix S(q,dq)
		//	-	float* q: a pointer to a 7-by-1 vector which must contain the current values of the 
		//		joint positions in radians
		//	-	float* dq: a pointer to a 7-by-1 vector which must contain the current values of the
		//		joint velocities in radians per second
		// outputs: none
		void get_S(float **S, float *q, float *dq, float* dyn_pars_tip = NULL);

		void get_S_fake(float **S, float *q, float *dq, float* dyn_pars_tip = NULL);

		// get_B returns the numerical computation of the current inertia matrix B(q)
		// inputs:
		//	-	float** B: a pointer to a 7-by-7 matrix, which will be filled with the numerical
		//		values of the inertia matrix B(q)
		//	-	float* q: a pointer to a 7-by-1 vector which must contain the current values of the 
		//		joint positions in radians
		// outputs: none
		void get_B(float** B, float* q, float* dyn_pars_tip = NULL);

		void get_B_fake(float** B, float* q, float* dyn_pars_tip = NULL);

		// get_c returns the numerical computation of the current Coriolis vector c(q,dq)
		// inputs:
		//	-	float* c: a pointer to a 7-by-1 vector, which will be filled with the numerical
		//		values of the Coriolis vector c(q,dq)
		//	-	float* q: a pointer to a 7-by-1 vector which must contain the current values of the 
		//		joint positions in radians
		//	-	float* dq: a pointer to a 7-by-1 vector which must contain the current values of the 
		//		joint velocities in radians/sec
		// outputs: none
		void get_c(float* c, float* q, float* dq, float* dyn_pars_tip = NULL);

		// get_g returns the numerical computation of the current gravity vector g(q)
		// inputs:
		//	-	float* g: a pointer to a 7-by-1 vector, which will be filled with the numerical
		//		values of the gravity vector g(q)
		//	-	float* q: a pointer to a 7-by-1 vector which must contain the current values of the 
		//		joint positions in radians
		// outputs: none
		//void get_g(float* g, float* q);

		// get_g returns the numerical computation of the current gravity vector g(q)
		// inputs:
		//	-	float* g: a pointer to a 7-by-1 vector, which will be filled with the numerical
		//		values of the gravity vector g(q)
		//	-	float* q: a pointer to a 7-by-1 vector which must contain the current values of the 
		//		joint positions in radians
		//	-	float* dyn_pars_tip: a pointer to a 10-by-1 vector, which contains the dynamic parameter
		//			of the tip in the modified DH convention (Craig). Please refer to the book
		//			"Modeling, Identification and Control of Robots" by Khalil, Dombre. The list is the following:
		//			[mass [Kg], cog_x [m], cog_y [m], cog_z [m], Jxx [Kg*m^2], Jxy [Kg*m^2], Jxz [Kg*m^2], Jyy [Kg*m^2], Jyz [Kg*m^2], Jzz [Kg*m^2]] 
		// outputs: none
		void get_g(float* g, float* q, float* dyn_pars_tip = NULL);

		void get_g_fake(float* g, float* q, float* dyn_pars_tip = NULL);

		// get_tau returns the numerical computation of the current torques vector tau(q,dq,ddq)
		// inputs:
		//	-	float* tau: a pointer to a 7-by-1 vector, which will be filled with the numerical
		//		values of the torques vector tau(q,dq,ddq)
		//	-	float* q: a pointer to a 7-by-1 vector which must contain the current values of the 
		//		joint positions in radians
		//	-	float* dq: a pointer to a 7-by-1 vector which must contain the current values of the 
		//		joint velocities in radians/sec
		//	-	float* ddq: a pointer to a 7-by-1 vector which must contain the current values of the 
		//		joint accelerations in radians/(sec^2)
		// outputs: none
		void get_tau(float* tau, float* q, float* dq, float* ddq, float* dyn_pars_tip = NULL);

		// to write...
		void get_friction(float *tau_f, float *dq);

	private:
		float** matrix_product(float** M1, float** M2, int row_m1, int col_m1, int col_m2);
		float* matrix_vector_product(float** M, float* v, int row_m, int col_m);
	};
