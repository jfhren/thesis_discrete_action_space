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

#include "uct.h"
#include "../../problems/generative_model.h"

uct_instance* uct_initInstance(state* initial, double discountFactor) {

    uct_instance* instance = (uct_instance*)malloc(sizeof(uct_instance));
    unsigned int i = 1;

    memset(instance, 0xda, sizeof(uct_instance));

    instance->gammaPowers[0] = 1.0;
    instance->bounds[0] = 1.0 / (1.0 - discountFactor);

    for(; i < UCT_MAX_DEPTH; i++) {
        instance->gammaPowers[i] = instance->gammaPowers[i - 1] * discountFactor;
        instance->bounds[i] = instance->gammaPowers[i] / (1.0 - discountFactor);
    }

    instance->gamma = discountFactor;
    instance->root = NULL;
    instance->totalNbEvaluations = 0;

    if(initial != NULL)
        uct_resetInstance(instance, initial);

    return instance;

}

static void deleteTree(uct_node *n) {

    if(n->children == NULL) {
        freeState(n->s);
        return;
    }

    uct_node* crt = n->children;

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


void uct_resetInstance(uct_instance* instance, state* initial) {

    if(instance->root != NULL) {
        deleteTree(instance->root);
        free(instance->root);
    }

    instance->root = (uct_node*)malloc(sizeof(uct_node));

    instance->root->s = copyState(initial);
    instance->root->reward = 0.0;
    instance->root->discountedSum = 0.0;
    instance->root->depth = 0;
    instance->root->n = 1;
    instance->root->crtOptimalLeaf = instance->root;
    instance->root->crtNextOpennedLeaf = NULL;
    instance->root->trajectoryId = 0;
    instance->root->id = K;
    instance->root->isClosedBranch = 0;
    instance->root->father = NULL;
    instance->root->children = NULL;

    instance->crtNbEvaluations = 0;
    instance->nextOpennedNode = instance->root;
    instance->crtOptimalAction = 0;
    instance->crtOptimalValue = 0.0;

}


static void buildingTrajectory(uct_instance* instance) {

    uct_node* n = instance->nextOpennedNode;
    unsigned int i = 0;

    n->children = (uct_node*)malloc(K * sizeof(uct_node));
    n->isClosedBranch = 1;

    n->n+=K;
    if(n == instance->crtOptimalLeaf)
        instance->crtOptimalValue = -1.0;

    n->trajectoryId = 0;

    for(;i < K; i++) {
        (n->children[i]).id = i;

        (n->children[i]).isClosedBranch = nextStateReward(n->s, actions[i], &((n->children[i]).s), &((n->children[i]).reward)) < 0 ? 1 : 0;
        instance->crtNbEvaluations++;
        instance->totalNbEvaluations++;
        instance->realNbEvaluations++;

        (n->children[i]).discountedSum = n->discountedSum + (instance->gammaPowers[n->depth] *  (n->children[i]).reward);
        (n->children[i]).crtOptimalLeaf = n->children + i;
        (n->children[i]).crtNextOpennedLeaf = n->children + i;

        if(n->depth == ((UCT_MAX_DEPTH) - 1))
            (n->children[i]).isClosedBranch = 1;

        if(!(n->children[i]).isClosedBranch) {
            if(n->isClosedBranch) {
                n->isClosedBranch = 0;
                n->crtOptimalLeaf = n->children + i;
                n->crtNextOpennedLeaf = n->children + i;
                n->trajectoryId = i;
            } else if((n->children[i]).discountedSum > n->crtOptimalLeaf->discountedSum) {
                n->crtOptimalLeaf = n->children + i;
                n->crtNextOpennedLeaf = n->children + i;
                n->trajectoryId = i;
            }
        }

        if((n->children[i]).discountedSum >= instance->crtOptimalValue) {
            instance->crtOptimalLeaf = n->children + i;
            instance->crtOptimalValue = (n->children[i]).discountedSum;
            instance->crtOptimalAction = instance->root->trajectoryId;
        }

        (n->children[i]).depth = n->depth + 1;
        (n->children[i]).n = 1;

        (n->children[i]).children = NULL;
        (n->children[i]).father = n;
    }

    n = n->father;

    while(n != NULL) {
        double crtMaxBound = 0.0;

        n->isClosedBranch = 1;
        n->n += K;

        for(i = 0; i < K; i++) {
            if(!(n->children[i]).isClosedBranch) {
                double crtBound = (n->children[i]).crtOptimalLeaf->discountedSum + (instance->bounds[n->depth] * sqrt(log(n->n) / (double)(n->children[i]).n));
                if(n->isClosedBranch) {
                    n->isClosedBranch = 0;
                    n->crtNextOpennedLeaf = (n->children[i]).crtNextOpennedLeaf;
                    n->crtOptimalLeaf = (n->children[i]).crtOptimalLeaf;
                    crtMaxBound = crtBound;
                    n->trajectoryId = i;
                } else {
                    if(crtBound > crtMaxBound) {
                        n->crtNextOpennedLeaf = (n->children[i]).crtNextOpennedLeaf;
                        crtMaxBound = crtBound;
                        n->trajectoryId = i;
                    }
                    if((n->children[i]).crtOptimalLeaf->discountedSum > n->crtOptimalLeaf->discountedSum)
                        n->crtOptimalLeaf = (n->children[i]).crtOptimalLeaf;
                }
            }
        }

        n = n->father;
    }

    instance->nextOpennedNode = instance->root->crtNextOpennedLeaf;

}


action* uct_planning(uct_instance* instance, unsigned int maxNbEvaluations) {

    instance->realNbEvaluations = 0;

    while((instance->crtNbEvaluations < maxNbEvaluations) && !instance->root->isClosedBranch)
        buildingTrajectory(instance);

    return actions[instance->crtOptimalAction];

}


static void updateValues(uct_instance* instance) {

    unsigned int crtDepth = 1;

    uct_node* crt = instance->root->children;

    while(1) {
        while(crt->children != NULL) {
            crt->discountedSum = crt->father->discountedSum + (instance->gammaPowers[crtDepth - 1] * crt->reward);
            crtDepth++;
            instance->crtNbEvaluations++;
            crt = crt->children;
        }

        instance->crtNbEvaluations++;
        crt->discountedSum = crt->father->discountedSum + (instance->gammaPowers[crtDepth - 1] * crt->reward);
        crt->depth = crtDepth;

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


static void updateCrtOptimalAction(uct_instance* instance) {

    uct_node* crt = instance->crtOptimalLeaf;

    while(crt->father->father)
        crt = crt->father;

    instance->crtOptimalAction = crt->id;

}


void uct_keepSubtree(uct_instance* instance) {

    if(instance->root->children) {
        unsigned int i = 0;
        unsigned int keptSubtreeId = instance->crtOptimalAction;
        uct_node* cuttedSubtrees = instance->root->children;

        freeState(instance->root->s);
        instance->root->s = (cuttedSubtrees[keptSubtreeId]).s;
        instance->root->reward = 0.0;
        instance->root->discountedSum = 0.0;
        instance->root->depth = 0;
        instance->root->n = (cuttedSubtrees[keptSubtreeId]).n;
        instance->root->trajectoryId = (cuttedSubtrees[keptSubtreeId]).trajectoryId;
        instance->root->id = K;
        instance->root->isClosedBranch = (cuttedSubtrees[keptSubtreeId]).isClosedBranch;
        instance->root->children = (cuttedSubtrees[keptSubtreeId]).children;

        for(; i < keptSubtreeId; i++)
            deleteTree(cuttedSubtrees + i);
        for(i = keptSubtreeId + 1; i < K; i++)
            deleteTree(cuttedSubtrees + i);
               
        instance->crtNbEvaluations = 0;

        if(instance->root->children == NULL) {
            instance->root->crtOptimalLeaf = instance->root;
            instance->root->crtNextOpennedLeaf = instance->root;
            instance->nextOpennedNode = instance->root;
            instance->crtOptimalLeaf = instance->root;
            instance->crtOptimalValue = 0.0;
            instance->crtOptimalAction = 0;
        } else {
            for(i = 0; i < K; i++)
                (instance->root->children[i]).father = instance->root;
            instance->root->crtOptimalLeaf = (cuttedSubtrees[keptSubtreeId]).crtOptimalLeaf;
            instance->root->crtNextOpennedLeaf = (cuttedSubtrees[keptSubtreeId]).crtNextOpennedLeaf;

            updateValues(instance);
            instance->nextOpennedNode = instance->root->crtNextOpennedLeaf;
            instance->crtOptimalLeaf = instance->root->crtOptimalLeaf;
            instance->crtOptimalValue = instance->crtOptimalLeaf->discountedSum;
            updateCrtOptimalAction(instance);
        }
        free(cuttedSubtrees);
    }

}


void uct_uninitInstance(uct_instance** instance) {

    deleteTree((*instance)->root);
    free((*instance)->root);

    free((*instance));
    *instance = NULL;

}


static unsigned int getMaxDepth(uct_node* crt) {

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


unsigned int uct_getMaxDepth(uct_instance* instance) {

    return instance->root->children ? getMaxDepth(instance->root->children) - 1 : 0;

}
