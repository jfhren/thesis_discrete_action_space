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

#include "optimistic.h"
#include "../../problems/generative_model.h"


static double crtDiscountedSums[OPTIMISTIC_MAX_DEPTH];



optimistic_instance* optimistic_initInstance(state* initial, double discountFactor) {

    optimistic_instance* instance = (optimistic_instance*)malloc(sizeof(optimistic_instance));
    unsigned int i = 1;

    memset(instance, 0xda, sizeof(optimistic_instance));

    instance->gammaPowers[0] = 1.0;
    instance->bounds[0] = 1.0 / (1.0 - discountFactor);

    for(; i < OPTIMISTIC_MAX_DEPTH; i++) {
        instance->gammaPowers[i] = instance->gammaPowers[i - 1] * discountFactor;
        instance->bounds[i] = instance->gammaPowers[i] / (1.0 - discountFactor);
    }

    instance->gamma = discountFactor;
    instance->root = NULL;
    instance->totalNbEvaluations = 0;

    if(initial != NULL)
        optimistic_resetInstance(instance, initial);

    return instance;

}

static void deleteTree(optimistic_node *n) {

    if(n->children == NULL) {
        free(n->values);

        freeState(n->s);

        return;
    }

    optimistic_node* crt = n->children;

    while(1) {
        while(crt->children != NULL)
            crt = crt->children;

        freeState(crt->s);
        free(crt->values);

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


void optimistic_resetInstance(optimistic_instance* instance, state* initial) {

    if(instance->root != NULL) {
        deleteTree(instance->root);
        free(instance->root);
    }

    instance->root = (optimistic_node*)malloc(sizeof(optimistic_node));

    instance->root->s = copyState(initial);

    instance->root->father = NULL;
    instance->root->children = NULL;

    instance->root->reward = 0.0;

    instance->root->values = malloc(sizeof(optimistic_node_values));
    ((optimistic_node_values*)instance->root->values)->bound = 0.0;
    ((optimistic_node_values*)instance->root->values)->discountedSum = 0.0;
    ((optimistic_node_values*)instance->root->values)->depth = 0;

    instance->root->id = K;
    instance->root->isClosedBranch = 0;

    instance->crtNbEvaluations = 0;
    instance->nextOpennedNode = instance->root;
    instance->crtOptimalAction = 0;
    instance->crtOptimalValue = 0.0;
    instance->root->trajectoryId = 0;
    instance->crtOptimalLeaf = instance->root;

}


static void buildingTrajectory(optimistic_instance* instance) {

    optimistic_node* n = instance->nextOpennedNode;                                         // The leaf that is going to be open now

    double crtDiscountedSum = ((optimistic_node_values*)n->values)->discountedSum;          // Let's take the current discounted sum of rewards for this trajectory

    unsigned int crtDepth = ((optimistic_node_values*)n->values)->depth;                    // The current depth of this leaf or its position in the trajectory

    unsigned int i = 0;

    n->children = (optimistic_node*)malloc(K * sizeof(optimistic_node));
    if(n == instance->crtOptimalLeaf)                                                       // If the current node being oponned is the current optimal then its first son will be the new current optimal one
        instance->crtOptimalValue = -1.0;

    n->isClosedBranch = 1;                                                                  // Let's suppose that every new children will be associated to a terminal state

    for(;i < K; i++) {
        (n->children[i]).id = i;
        (n->children[i]).trajectoryId = 0;

        (n->children[i]).isClosedBranch = nextStateReward(n->s, actions[i], &((n->children[i]).s), &((n->children[i]).reward)) < 0 ? 1 : 0;
        instance->crtNbEvaluations++;
        instance->totalNbEvaluations++;
        instance->realNbEvaluations++;

        if(crtDepth == ((OPTIMISTIC_MAX_DEPTH) - 1))
            (n->children[i]).isClosedBranch = 1;

        (n->children[i]).values = (i == 0) ? n->values : malloc(sizeof(optimistic_node_values));    // The first children get its father values, the others get new ones

        ((optimistic_node_values*)(n->children[i]).values)->discountedSum = crtDiscountedSum + (instance->gammaPowers[crtDepth] *  (n->children[i]).reward);   // Actualization of the discounted sum of rewards

        ((optimistic_node_values*)(n->children[i]).values)->bound = ((optimistic_node_values*)(n->children[i]).values)->discountedSum + instance->bounds[crtDepth + 1];     // Computation of the bound for this new leaf

        if(!(n->children[i]).isClosedBranch) {                                              // If this leaf is not closed
            if(n->isClosedBranch) {                                                         // If my father still supposes all its children will be closed then...
                n->isClosedBranch = 0;                                                      // ...it's not the case, father
                n->values = (void*)(n->children + i);                                       // Let's put this children as a reference for the max bound
                n->trajectoryId = i;
            } else if(((optimistic_node_values*)((optimistic_node*)n->values)->values)->bound < ((optimistic_node_values*)(n->children[i]).values)->bound) {    // Else if this children bound is the biggest then...
                n->values = (void*)(n->children + i);                                       // ...let's put if as the reference for the max bound
                n->trajectoryId = i;
            }
        }

        if(((optimistic_node_values*)(n->children[i]).values)->discountedSum > instance->crtOptimalValue) {     // If it's the best then let's save its value, its address and the root action that was taken
            instance->crtOptimalValue = ((optimistic_node_values*)(n->children[i]).values)->discountedSum;
            instance->crtOptimalLeaf = n->children + i;
            instance->crtOptimalAction = instance->root->trajectoryId;
        }

        ((optimistic_node_values*)(n->children[i]).values)->depth = crtDepth + 1;

        (n->children[i]).children = NULL;
        (n->children[i]).father = n;
    }

    n = n->father;                                                                          // Let's update the max overal bound starting from the father of the openned leaf (which is not one anymore)

    while(n != NULL) {

        unsigned int i = 0;

        n->isClosedBranch = 1;                                                              // As before let's suppose this node is closed

        for(; i < K; i++) {
            if(!(n->children[i]).isClosedBranch) {                                          // If one of its children is not closed
                if(n->isClosedBranch) {                                                     // If every children was closed but not this one
                    n->isClosedBranch = 0;                                                  // Then this node is not closed

                    if((n->children[i]).children == NULL)                                   // If this child is a leaf
                        n->values = (void*)(n->children + i);
                    else
                        n->values = (n->children[i]).values;

                    n->trajectoryId = i;

                } else if((n->children[i]).children == NULL) {                              // Else this node is not closed anyway so is this child a leaf ?
                    if(((optimistic_node_values*)(n->children[i]).values)->bound > ((optimistic_node_values*)((optimistic_node*)(n->values))->values)->bound) {
                        n->values = (void*)(n->children + i);
                        n->trajectoryId = i;
                    }
                } else {
                    if(((optimistic_node_values*)((optimistic_node*)(n->children[i]).values)->values)->bound > ((optimistic_node_values*)((optimistic_node*)(n->values))->values)->bound) {
                        n->values = (n->children[i]).values;
                        n->trajectoryId = i;
                    }
                }
            }
        }

        n = n->father;                                                                      // Update done for this node, let's move on to its father
    }

    instance->nextOpennedNode = (optimistic_node*)instance->root->values;                   // The next leaf to be openned

}


action* optimistic_planning(optimistic_instance* instance, unsigned int maxNbEvaluations) {

    unsigned int cpt = 15000000;
    instance->realNbEvaluations = 0;

    while((instance->crtNbEvaluations < maxNbEvaluations) && !instance->root->isClosedBranch) {
        if(instance->crtNbEvaluations > cpt){
            printf("%u evaluations done\n", cpt);
            cpt+=15000000;
        }

        buildingTrajectory(instance);
    }

    return actions[instance->crtOptimalAction];

}


static void updateValues(optimistic_instance* instance) {

    unsigned int crtDepth = 1;

    optimistic_node* crt = instance->root->children;
    instance->crtOptimalValue = 0.0;
    crtDiscountedSums[0] = 0.0;

    while(1) {
        while(crt->children != NULL) {
            crtDiscountedSums[crtDepth] = crtDiscountedSums[crtDepth-1] + (instance->gammaPowers[crtDepth - 1] * crt->reward);
            crtDepth++;
            instance->crtNbEvaluations++;
            crt = crt->children;
        }

        instance->crtNbEvaluations++;
        ((optimistic_node_values*)crt->values)->discountedSum = crtDiscountedSums[crtDepth-1] + (instance->gammaPowers[crtDepth - 1] * crt->reward);

        if(((optimistic_node_values*)crt->values)->discountedSum > instance->crtOptimalValue) {
            instance->crtOptimalValue = ((optimistic_node_values*)crt->values)->discountedSum;
            instance->crtOptimalLeaf = crt;
        }

        ((optimistic_node_values*)crt->values)->bound = ((optimistic_node_values*)crt->values)->discountedSum + instance->bounds[crtDepth];
        ((optimistic_node_values*)crt->values)->depth = crtDepth;

        while(crt && (crt->id >= (K - 1))) {
            crtDepth--;
            crt = crt->father;
        }

        if(crt)
            crt = crt->father->children + crt->id + 1;
        else
            return;
    }

}


static void updateCrtOptimalAction(optimistic_instance* instance) {

    instance->crtOptimalValue = ((optimistic_node_values*)instance->crtOptimalLeaf->values)->discountedSum;
    optimistic_node* crt = instance->crtOptimalLeaf;

    while(crt->father->father)
        crt = crt->father;

    instance->crtOptimalAction = crt->id;

}


static void updateNextOpennedNode(optimistic_instance* instance) {

    optimistic_node* crt = instance->root;

    while(crt->children != NULL)
        crt = crt->children + crt->trajectoryId;

    instance->nextOpennedNode = crt;

}


void optimistic_keepSubtree(optimistic_instance* instance) {

    if(instance->root->children) {
        unsigned int i = 0;
        unsigned int keptSubtreeId = instance->crtOptimalAction;
        optimistic_node* cuttedSubtrees = instance->root->children;

        freeState(instance->root->s);
        instance->root->s = (cuttedSubtrees[keptSubtreeId]).s;
        instance->root->reward = 0.0;
        instance->root->values = (cuttedSubtrees[keptSubtreeId]).values;
        instance->root->trajectoryId = (cuttedSubtrees[keptSubtreeId]).trajectoryId;
        instance->root->children = (cuttedSubtrees[keptSubtreeId]).children;

        for(; i < keptSubtreeId; i++)
            deleteTree(cuttedSubtrees + i);
        for(i = keptSubtreeId + 1; i < K; i++)
            deleteTree(cuttedSubtrees + i);
        free(cuttedSubtrees);
        
        instance->crtOptimalValue = 0.0;
        instance->crtNbEvaluations = 0;

        if(instance->root->children == NULL) {
            instance->nextOpennedNode = instance->root;
            instance->crtOptimalAction = 0;
            instance->crtOptimalLeaf = instance->root;
        } else {
            for(i = 0; i < K; i++)
                (instance->root->children[i]).father = instance->root;
            updateValues(instance);
            updateNextOpennedNode(instance);
            updateCrtOptimalAction(instance);
        }
    }

}


void optimistic_uninitInstance(optimistic_instance** instance) {

    deleteTree((*instance)->root);
    free((*instance)->root);

    free((*instance));
    *instance = NULL;

}


static unsigned int getMaxDepth(optimistic_node* crt) {

    unsigned int crtDepth = 1;
    unsigned int maxDepth = 0;

    while(1) {
        while(crt->children != NULL) {
            crtDepth++;
            crt = crt->children;
        }

        if(crtDepth > maxDepth)
            maxDepth = crtDepth;

        while(crt && (crt->id >= (K - 1))) {
            crtDepth--;
            crt = crt->father;
        }

        if(crt)
            crt = crt->father->children + crt->id + 1;
        else
            break;
    }

    return maxDepth;

}


unsigned int optimistic_getMaxDepth(optimistic_instance* instance) {

    return instance->root->children ? getMaxDepth(instance->root->children) - 1 : 0;

}
