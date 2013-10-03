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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>

#include <gsl/gsl_rng.h>

#include "random_search.h"
#include "../../problems/generative_model.h"


random_search_instance* random_search_initInstance(state* initial, double discountFactor) {

    unsigned int i = 1;
    random_search_instance* instance = (random_search_instance*)malloc(sizeof(random_search_instance));

    instance->QValues = NULL;
    instance->rng = NULL;
    instance->trajectories = NULL;
    instance->initial = NULL;

    instance->gamma = discountFactor;
    instance->gammaPowers[0] = 1.0;
    for(;i < RANDOM_SEARCH_MAX_DEPTH; i++)
        instance->gammaPowers[i] = instance->gammaPowers[i - 1] * discountFactor;

    if(initial != NULL)
        random_search_resetInstance(instance, initial);

    return instance;

}


static void deleteTrajectories(random_search_instance* instance) {

    random_search_trajectory* crtTrajectory = instance->trajectories;

    while(crtTrajectory != NULL) {
        random_search_node* crt = crtTrajectory->trajectory;
        random_search_trajectory* tmpTrajectory = crtTrajectory->next;
        while(crt != NULL) {
            random_search_node* tmp = crt->next;
            freeState(crt->s);
            free(crt);
            crt = tmp;
        }
        free(crtTrajectory);
        crtTrajectory = tmpTrajectory;
    }

    instance->trajectories = NULL;

}


void random_search_resetInstance(random_search_instance* instance, state* initial) {

    if(instance->QValues == NULL) {
        instance->QValues = (double*)malloc(sizeof(double) * K);
        instance->rng = gsl_rng_alloc(gsl_rng_mt19937);
    } else {
        if(instance->initial != NULL)
            freeState(instance->initial);
        deleteTrajectories(instance);
    }

    memset(instance->QValues, 0, sizeof(double) * K);
    gsl_rng_set(instance->rng, time(NULL));

    instance->initial = copyState(initial);

    instance->crtDepthLimit = 1;
    instance->crtNbEvaluations = 0;
    instance->crtMaxDepth = 0;
    instance->crtOptimalValue = 0.0;
    instance->crtOptimalAction = 0;

}


action* random_search_planning(random_search_instance* instance, unsigned int maxNbEvaluations) {

    while(instance->crtNbEvaluations < maxNbEvaluations) {
        random_search_trajectory* newTrajectory = (random_search_trajectory*)malloc(sizeof(random_search_trajectory));
        random_search_node* crtNode = (random_search_node*)malloc(sizeof(random_search_node));
        random_search_node* prevNode = NULL;
        state* crt = instance->initial;
        state* next = NULL;
        double reward = 0.0;
        unsigned int firstAction = gsl_rng_uniform_int(instance->rng, K);
        unsigned int crtDepth = 1;
        double discountedSum = 0.0;

        newTrajectory->next = instance->trajectories;
        instance->trajectories = newTrajectory;

        newTrajectory->trajectory = crtNode;
        crtNode->s = copyState(crt);
        crtNode->reward = 0.0;
        crtNode->next = NULL;

        nextStateReward(crt, actions[firstAction], &next, &discountedSum);
        instance->crtNbEvaluations++;

        crt = next;

        prevNode = crtNode;
        crtNode = (random_search_node*)malloc(sizeof(random_search_node));
        crtNode->s = copyState(crt);
        crtNode->reward = discountedSum;
        crtNode->next = NULL;
        prevNode->next = crtNode;

        while(crtDepth <= instance->crtDepthLimit) {
            char isTerminal = nextStateReward(crt, actions[gsl_rng_uniform_int(instance->rng, K)], &next, &reward) < 0 ? 1 : 0;
            instance->crtNbEvaluations++;
            discountedSum += instance->gammaPowers[crtDepth] * reward;

            freeState(crt);
            crt = next;

            prevNode = crtNode;
            crtNode = (random_search_node*)malloc(sizeof(random_search_node));
            crtNode->s = copyState(crt);
            crtNode->reward = reward;
            crtNode->next = NULL;
            prevNode->next = crtNode;

            if(isTerminal)
                break;

            crtDepth++;
        }

        freeState(crt);

        if(instance->crtDepthLimit > instance->crtMaxDepth)
            instance->crtMaxDepth = instance->crtDepthLimit;

        if(discountedSum > instance->QValues[firstAction]) {
            instance->QValues[firstAction] = discountedSum;
            if(discountedSum > instance->crtOptimalValue) {
                instance->crtOptimalValue = discountedSum;
                instance->crtOptimalAction = firstAction;
            }
        }

        if(instance->crtDepthLimit < (RANDOM_SEARCH_MAX_DEPTH - 1)) {
            instance->crtDepthLimit = (unsigned int)(log(instance->crtNbEvaluations) / log(1.0/instance->gamma));
            if(instance->crtDepthLimit >= RANDOM_SEARCH_MAX_DEPTH)
                instance->crtDepthLimit = RANDOM_SEARCH_MAX_DEPTH - 1;
            if(instance->crtDepthLimit < 1)
                instance->crtDepthLimit = 1;
        }
    }

    return actions[instance->crtOptimalAction];

}


unsigned int random_search_getMaxDepth(random_search_instance* instance) {

    return instance->crtMaxDepth - 1;

}


void random_search_uninitInstance(random_search_instance** instance) {

    free((*instance)->QValues);
    gsl_rng_free((*instance)->rng);
    freeState((*instance)->initial);
    deleteTrajectories(*instance);

    free(*instance);
    *instance = NULL;

}
