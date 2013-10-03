/* Copyright or © or Copr. 2012, Jean-François Hren
 *
 * Author e-mail: jean-francois.hren@inria.fr
 *
 * This software is a computer program whose purpose is to control
 * deterministic systems using optimistic planning.
 *
 * This software is governed by the CeCILL license under French law and
 * abiding by the rules of distribution of free software.  You can  use, 
 * modify and/ or redistribute the software under the terms of the CeCILL
 * license as circulated by CEA, CNRS and INRIA at the following URL
 * "http://www.cecill.info". 
 *
 * As a counterpart to the access to the source code and  rights to copy,
 * modify and redistribute granted by the license, users are provided only
 * with a limited warranty  and the software's author,  the holder of the
 * economic rights,  and the successive licensors  have only  limited
 * liability. 
 *
 * In this respect, the user's attention is drawn to the risks associated
 * with loading,  using,  modifying and/or developing or reproducing the
 * software by the user in light of its specific status of free software,
 * that may mean  that it is complicated to manipulate,  and  that  also
 * therefore means  that it is reserved for developers  and  experienced
 * professionals having in-depth computer knowledge. Users are therefore
 * encouraged to load and test the software's suitability as regards their
 * requirements in conditions enabling the security of their systems and/or 
 * data to be ensured and,  more generally, to use and operate it in the 
 * same conditions as regards security. 
 *
 * The fact that you are presently reading this means that you have had
 * knowledge of the CeCILL license and that you accept its terms.
 */

#include <stdio.h>
#include <stdlib.h>
#define __USE_GNU
#include <math.h>
#undef __USE_GNU
#include <string.h>

#include "mountain_car.h"

unsigned int K = 2;

double timeStep = 0.1;                                /*Time step between two state*/

action** actions = NULL;

double* parameters = NULL;                            /*Model's parameters*/
unsigned int nbParameters = 4;                        /*Number of model's parameters */

/*+--------Model's parameters-------+
  |                                 |
  | parameters[0]: Gravity          |
  | parameters[1]: Car mass         |
  | parameters[2]: Max velocity     |
  | parameters[3]: Max acceleration |
  |                                 |
  +---------------------------------+*/


/* Initialisation of the parameters. To call before anything else.*/

void initGenerativeModelParameters() {

    parameters = (double*)malloc(sizeof(double) * nbParameters);

    parameters[0] = 9.81;

    parameters[1] = 1.0;

    parameters[2] = 3;

    parameters[3] = 4;

}


/* Initialisation of the generative model. To call after parameters initialisation. */

void initGenerativeModel() {

	unsigned int i = 0;

    actions = (action**)malloc(sizeof(action*) * K);
		
	for(; i < K; i++) {
			actions[i] = (action*)malloc(sizeof(action));
			actions[i]->xAcceleration = -parameters[3] + (((parameters[3] + parameters[3]) / (float)(K - 1)) * i);
	}

}


/* Free the generative model. To call if the generative model has to be discarded. */

void freeGenerativeModel() {

    unsigned int i = 0;

    for(;i < K; i++)
        free(actions[i]);

    free(actions);

}


/* Free the generative model parameters. To call afer everything is finished. */

void freeGenerativeModelParameters() {

    free(parameters);

}


/* Returns an allocated state initialized from the parsed string */
state* makeState(const char* str) {

    char* end = NULL;
    char* crt = (char*)str;
    char tmp[255];    
    state* s = (state*)malloc(sizeof(state));

    end = strchr(str, ',');
    memcpy(tmp, crt, end - crt);
    tmp[end-crt] = '\0';
    s->xPosition = strtod(tmp, NULL);
    crt = end + 1;

    s->xVelocity = strtod(crt, NULL);

    s->isTerminal = 0;

    return s;


}


/* Returns an allocated initial state of the model. */

state* initState() {

    state* initial = (state*)malloc(sizeof(state));

    initial->xPosition = -0.5;    /*Position of the car*/

    initial->xVelocity = 0.0;     /*Velocity of the car*/

    initial->isTerminal = 0;

    return initial;

}


/* Returns a triplet containing the next state, the applied action and the reward given the current state and action. */

char nextStateReward(state* s, action* a, state** nextState, double* reward) {
    *nextState = copyState(s);

    if(s->isTerminal < 0) {
        *reward = 0.0;
    } else if(s->isTerminal > 0) {
        *reward = 1.0;
    } else {
        unsigned int i = 0;
        for(; i < 100; i++) {
            double deltaPosition = 0.001 * (*nextState)->xVelocity;

            double xSquare = (*nextState)->xPosition * (*nextState)->xPosition;
            double hillPrime = (*nextState)->xPosition < 0.0 ? (2.0 * (*nextState)->xPosition) + 1.0 : sqrt(1.0 + (5.0 * xSquare)) / ((25.0 * xSquare * xSquare) + (10.0 * xSquare) + 1.0);
            double hillPrimePrime = (*nextState)->xPosition < 0.0 ? 2.0 : -(15.0 * (*nextState)->xPosition * sqrt(1.0 + (5.0 * xSquare)) / ((125.0 * xSquare * xSquare * xSquare) + (75.0 * xSquare * xSquare) + (15.0 * xSquare) + 1));
            double hillPrimeSquare = 1 + (hillPrime * hillPrime);

            double deltaVelocity = 0.001 * ((a->xAcceleration / (parameters[1] * hillPrimeSquare)) - (parameters[0] * hillPrime / hillPrimeSquare) - ((*nextState)->xVelocity * (*nextState)->xVelocity * hillPrime * hillPrimePrime / hillPrimeSquare));

            (*nextState)->xPosition += deltaPosition;
            (*nextState)->xVelocity += deltaVelocity;
        }

        if(fabs((*nextState)->xVelocity) > parameters[2])
            (*nextState)->isTerminal = -1;
        else {
            if((*nextState)->xPosition >= 1.0)
                (*nextState)->isTerminal = 1;
            if((*nextState)->xPosition < -1.0)
                (*nextState)->isTerminal = -1;
        }

        if((*nextState)->isTerminal < 0)
            *reward = 0.0;
        else if((*nextState)->isTerminal > 0)
            *reward = 1.0;
        else
            *reward = ((*nextState)->xPosition + 1) / 2.0;
    }

    return (*nextState)->isTerminal;

}


/* Returns the id corresponding to the place of the action a in the array actions. */

unsigned int getActionId(action* a) {

    unsigned int id = 0;

    while(a != actions[id])
        id++;

    return id;

}


/* Returns an allocated copy of the state s */

state* copyState(state* s) {

    state* newState = (state*)malloc(sizeof(state));
    memcpy(newState, s, sizeof(state));

    return newState;

}


void printState(state* s) {

    printf("xPosition: %f ",s->xPosition);

}


void printAction(action* a) {

    printf("action: %f ", a->xAcceleration);

}


/* Free the state s */

void freeState(state* s){

    free(s);

}
