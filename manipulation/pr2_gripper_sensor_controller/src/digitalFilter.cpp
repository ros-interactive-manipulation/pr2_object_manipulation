/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joeromano@gmail.com> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return
 * Joe Romano and Will McMahan
 * ----------------------------------------------------------------------------
 */
//@author  Joe Romano
//@author  Will McMahan
//@email   joeromano@gmail.com
//@brief   digitalFilter.cpp - class to create IIR and FIR digital filter
//         coefficients in the Matlab (2010) style. This style being vectors
//         of coefficients for the numberator (b) and denominator (a) 
//         respectively. 
//         Please refer to the matlab documentation page for implementation
//         details: http://www.mathworks.com/access/helpdesk/help/techdoc/ref/filter.html

#include "pr2_gripper_sensor_controller/digitalFilter.h"

digitalFilter::digitalFilter(int filterOrder_userdef, bool isIIR)
{
	filterOrder = filterOrder_userdef;
        IIR = isIIR;
	
	b = new float [filterOrder + 1];
	a = new float [filterOrder + 1];

	x = new float [filterOrder + 1];
	u = new float [filterOrder + 1];

	// Initialize the arrays with zeros
	for(int i = 0; i < (filterOrder + 1); i++)
	{
		b[i] = 0.0;
		a[i] = 0.0;
		x[i] = 0.0;
		u[i] = 0.0;
	}
}	

digitalFilter::digitalFilter(int filterOrder_userdef, bool isIIR, float *b_userdef, float *a_userdef)
{
  
	filterOrder = filterOrder_userdef;
        IIR = isIIR;
	
	b = new float [filterOrder + 1];
	a = new float [filterOrder + 1];

	x = new float [filterOrder + 1];
	u = new float [filterOrder + 1];

	// Initialize the arrays
	
	for(int i = 0; i < (filterOrder + 1); i++)
	{
		b[i] = b_userdef[i];
		a[i] = a_userdef[i];
		x[i] = 0.0;
		u[i] = 0.0;
	}
	
}

float digitalFilter::getNextFilteredValue(float u_current)
{
	/* Shift x2 and u2 vectors, losing the last elements and putting new u2 value in zeroth spot. */
	for (int i = filterOrder ; i > 0 ; i--) {
		x[i] = x[i-1];
		u[i] = u[i-1];
	}
	u[0] = u_current; 

	/* Simulate system. */
	float output = b[0] * u[0];
	  
        // if we have an IIR filter            
        if(IIR)
        {
            for (int i = 1 ; i < (filterOrder+1) ; i++) {
                    output += b[i] * u[i] - a[i] * x[i];
            }
        }

       // if we have an FIR filter
       else
       {
            for (int i = 1 ; i < (filterOrder+1) ; i++) {
                    output += b[i] * u[i];
            }
       }

	/* Put the result in shared memory and in the x2 vector. */
	x[0] = output;

	return output;
}

digitalFilter::~digitalFilter(void)
{
	delete[] x;
	delete[] u;
	delete[] a;
	delete[] b;
}
