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
#include <string.h>

#include "uniform.h"
#include "../../problems/generative_model.h"

uniform_instance* uniform_initInstance(state* initial, double discountFactor) {

    uniform_instance* instance = (uniform_instance*)malloc(sizeof(uniform_instance));
    unsigned int i = 1;

    instance->gammaPowers[0] = 1.0;

    for(; i < UNIFORM_MAX_DEPTH; i++)
        instance->gammaPowers[i] = instance->gammaPowers[i - 1] * discountFactor;

    instance->gamma = discountFactor;
    instance->root = NULL;
    instance->totalNbEvaluations = 0;

    if(initial != NULL)
        uniform_resetInstance(instance, initial);

    return instance;

}

static void deleteTree(uniform_node *n) {

    if(n->children == NULL) {
        freeState(n->s);

        return;
    }

    uniform_node* crt = n->children;

    while(1) {
        while(crt->children != NULL)
            crt = crt->children;

        freeState(crt->s);

        while((crt != n) && (crt->id >= (K - 1))) {
            crt = crt->father;

            if(crt) {
                free(crt->children);
                freeState(crt->s);
            }
        }

        if(crt != n)
            crt = crt->father->children + crt->id + 1;
        else
            return;
    }

}


void uniform_resetInstance(uniform_instance* instance, state* initial) {

    if(instance->root != NULL) {
        deleteTree(instance->root);
        free(instance->root);
    }

    instance->root = (uniform_node*)malloc(sizeof(uniform_node));

    instance->root->s = copyState(initial);

    instance->root->father = NULL;
    instance->root->children = NULL;

    instance->root->reward = 0.0;
    instance->root->discountedSum = 0.0;
    instance->root->id = K;
    instance->root->trajectoryId = 0;

    instance->root->crtOptimalLeaf = instance->root;

    instance->crtNbEvaluations = 0;
    instance->nextOpennedNode = instance->root;

    instance->crtDepth = 0;

}


static void updateNextOpennedNode(uniform_instance* instance) {

    uniform_node* crt = instance->nextOpennedNode->father;
    uniform_node* prev = instance->nextOpennedNode;

    while((crt != NULL) && ((crt->children + (K - 1)) == prev)) {
        prev = crt;
        crt = crt->father;
    }

    if(crt == NULL) {
        crt = instance->root;
        instance->crtDepth++;
    } else {
        crt = crt->children + prev->id + 1;
    }

    while(crt->children != NULL)
        crt = crt->children;

    instance->nextOpennedNode = crt;

}


static void buildingTrajectory(uniform_instance* instance) {

    uniform_node* n = instance->nextOpennedNode;

    unsigned int i = 0;

    n->children = (uniform_node*)malloc(K * sizeof(uniform_node));

    n->crtOptimalLeaf = n->children;
    n->trajectoryId = 0;    

    for(;i < K; i++) {
        (n->children[i]).id = i;
        (n->children[i]).trajectoryId = 0;

        nextStateReward(n->s, actions[i], &((n->children[i]).s), &((n->children[i]).reward));
        instance->crtNbEvaluations++;
        instance->totalNbEvaluations++;
        instance->realNbEvaluations++;

        (n->children[i]).discountedSum = n->discountedSum + (instance->gammaPowers[instance->crtDepth] *  (n->children[i]).reward);

        if((n->children[i]).discountedSum > n->crtOptimalLeaf->discountedSum) {
            n->crtOptimalLeaf = n->children + i;
            n->trajectoryId = i;
        }

        (n->children[i]).children = NULL;
        (n->children[i]).father = n;

        (n->children[i]).crtOptimalLeaf = n->children + i;
    }

    n = n->father;

    while(n != NULL) {
        n->crtOptimalLeaf = n->children->crtOptimalLeaf;
        n->trajectoryId = 0;
        for(i = 1; i < K; i++) {
            if((n->children[i]).crtOptimalLeaf->discountedSum > n->crtOptimalLeaf->discountedSum) {
                n->crtOptimalLeaf = (n->children[i]).crtOptimalLeaf;
                n->trajectoryId = i;
            }
        }

        n = n->father;
    }

    updateNextOpennedNode(instance);

}


action* uniform_planning(uniform_instance* instance, unsigned int maxNbEvaluations) {

    instance->realNbEvaluations = 0;

    while((instance->crtNbEvaluations < maxNbEvaluations) && (instance->crtDepth < (UNIFORM_MAX_DEPTH - 1)))
        buildingTrajectory(instance);

    return actions[instance->root->trajectoryId];

}


static void updateValues(uniform_instance* instance) {

    unsigned int crtDepth = 1;
    unsigned int crtMaxDepth = 1;

    uniform_node* crt = instance->root->children;

    while(1) {
        while(crt->children != NULL) {
            crt->discountedSum = crt->father->discountedSum + (instance->gammaPowers[crtDepth - 1] * crt->reward);
            crtDepth++;
            instance->crtNbEvaluations++;
            crt = crt->children;
        }

        instance->crtNbEvaluations++;
        crt->discountedSum = crt->father->discountedSum + (instance->gammaPowers[crtDepth - 1] * crt->reward);

        if(crtDepth >= crtMaxDepth) {
            crtMaxDepth = crtDepth;
            instance->nextOpennedNode = crt->father;
        }

        while(crt && (crt->id >= (K - 1))) {
            crtDepth--;
            crt = crt->father;
        }

        if(crt)
            crt = crt->father->children + crt->id + 1;
        else
            break;
    }

    instance->crtDepth = crtMaxDepth;

}


void uniform_keepSubtree(uniform_instance* instance) {

    if(instance->root->children) {
        unsigned int i = 0;
        unsigned int keptSubtreeId = instance->root->trajectoryId;
        uniform_node* cuttedSubtrees = instance->root->children;

        freeState(instance->root->s);
        instance->root->s = (cuttedSubtrees[keptSubtreeId]).s;
        instance->root->trajectoryId = (cuttedSubtrees[keptSubtreeId]).trajectoryId;
        instance->root->children = (cuttedSubtrees[keptSubtreeId]).children;

        for(; i < keptSubtreeId; i++)
            deleteTree(cuttedSubtrees + i);
        for(i = keptSubtreeId + 1; i < K; i++)
            deleteTree(cuttedSubtrees + i);
        free(cuttedSubtrees);
        
        instance->crtNbEvaluations = 0;

        if(instance->root->children == NULL) {
            instance->root->crtOptimalLeaf = instance->root;
            instance->nextOpennedNode = instance->root;
            instance->crtDepth = 0;
        } else {
            for(i = 0; i < K; i++)
                (instance->root->children[i]).father = instance->root;
            updateValues(instance);
            updateNextOpennedNode(instance);
        }

        //printf("%u evaluations kept\n", instance->crtNbEvaluations);
    }
}


unsigned int uniform_getMaxDepth(uniform_instance* instance) {

    uniform_node* crt = instance->root;
    unsigned int depth = 0;

    while(crt && (crt->children != NULL)) {
        crt = crt->children;
        depth++;
    }

    return depth;

}


void uniform_uninitInstance(uniform_instance** instance) {

    deleteTree((*instance)->root);
    free((*instance)->root);

    free((*instance));
    *instance = NULL;

}
