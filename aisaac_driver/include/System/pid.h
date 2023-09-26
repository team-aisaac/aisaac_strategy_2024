/*
 * pid.h
 *
 *  Created on: 2021/09/11
 *      Author: 三宅章太
 */

#ifndef PID_H_
#define PID_H_

typedef struct {
	/***parameter***/
	float t;	//time cycle[ms]

	float p;	//gain p[META]
	float i;	//gain i[META]
	float d;	//gain d[META]

	float outLimit;	//[META]
	float integralOutLimit;	//[META]
	float cutoffFrequency; //[Hz]
	/***parameter***/

	/***calculate***/
	float integralOut;	//[META]
	float lastError;	//[META]
	float differentialFilter;	//[META]
	float differentialFilterRate;	//[META]
	/***calculate***/

	/***input***/
	float error;	//[META]
	/***input***/

} _pid_t;

//calculate output value from error
float pidExecute(_pid_t *pid);

#endif /* PID_H_ */
