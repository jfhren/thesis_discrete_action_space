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
#include <math.h>
#include <string.h>

#include "ball.h"

unsigned int K = 2;                             //Number of actions that can be applied in a state
double timeStep = 0.1;                          //Time step between two state

action** actions = NULL;                        //Array of action

double* parameters = NULL;                      //Model's parameters
unsigned int nbParameters = 3;                  //Number of model's parameters

/*+------------Model's parameters------------+
  |                                          |
  | parameters[0] : half-lenght of the track |
  | parameters[1] : maximum acceleration     |
  | parameters[2] : maximum velocity         |
  |                                          |
  +------------------------------------------+*/


/* Initialisation of the parameters. To call before anything else.*/

void initGenerativeModelParameters() {

    parameters = (double*)malloc(sizeof(double) * nbParameters);

    parameters[0] = 1.0;

    parameters[1] = 1.0;

    parameters[2] = 10.0;

}


/* Initialisation of the generative model. To call after parameters initialisation. */

void initGenerativeModel() {

    unsigned int i = 0;

    actions = (action**)malloc(sizeof(action*) * K);
		
	for(; i < K; i++) {
			actions[i] = (action*)malloc(sizeof(action));
			actions[i]->acceleration = -parameters[1] + (((parameters[1] + parameters[1]) / (float)(K - 1)) * i);
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
    s->position = strtod(tmp, NULL);
    crt = end + 1;

    s->velocity = strtod(crt, NULL);
    
    s->isTerminal = 0;

    return s;


}


/* Returns an allocated initial state of the model. */

state* initState() {

    state* init = (state*)malloc(sizeof(state));

    init->position = -parameters[0];        //Position of the ball

    init->velocity = 0.0;                   //Velocity of the ball

    init->isTerminal = 0;

    return init;

}


/* Returns a triplet containing the next state, the applied action and the reward given the current state and action. */

char nextStateReward(state* s, action* a, state** nextState,double* reward) {

    *reward = 0.0;

    state* newState = (state*)malloc(sizeof(state));

    newState->position = s->position;

    newState->velocity = s->velocity;

    newState->velocity += (a->acceleration * timeStep);

    if(fabs(newState->velocity) > parameters[2])
        newState->velocity = newState->velocity > 0.0 ? parameters[2] : -parameters[2];

    newState->position += (newState->velocity * timeStep);

/*    if(fabs(newState->position) > parameters[0]) {
        newState->isTerminal = -1;
    } else {*/
        *reward = 1 - pow(newState->position, 2);

        if(*reward < 0.0)
            *reward = 0.0;

        if(*reward > 1.0)
            *reward = 1.0;

        newState->isTerminal = 0;
//    }

    *nextState = newState;

    return newState->isTerminal;

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

    newState->position = s->position;
    newState->velocity = s->velocity;
    newState->isTerminal = s->isTerminal;

    return newState;

}


void printState(state* s) {

    printf("xPosition: %f ",s->position);

}


void printAction(action* a) {

    printf("action: %f ", a->acceleration);

}


/* Free the state s */

void freeState(state* s){

    free(s);

}
