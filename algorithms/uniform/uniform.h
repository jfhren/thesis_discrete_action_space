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

#ifndef UNIFORM_H
#define UNIFORM_H

#ifdef LIMITED_DEPTH
#define UNIFORM_MAX_DEPTH 512
#else
#define UNIFORM_MAX_DEPTH 32768
#endif

#include "../../problems/generative_model.h"

typedef struct uniform_node_struct {
        state* s;
        double reward;
        unsigned int id;
        double discountedSum;
        unsigned int trajectoryId;
        struct uniform_node_struct* crtOptimalLeaf;
        struct uniform_node_struct* father;
        struct uniform_node_struct* children;
}	uniform_node;

typedef struct {

        double gamma;
        uniform_node* root;

        unsigned int crtNbEvaluations;
        unsigned int realNbEvaluations;
        unsigned int totalNbEvaluations;

        unsigned int crtDepth;

        double gammaPowers[UNIFORM_MAX_DEPTH];

        uniform_node* nextOpennedNode;

}   uniform_instance;


uniform_instance* uniform_initInstance(state* initial, double discountFactor);
void uniform_resetInstance(uniform_instance* instance, state* initial);
action* uniform_planning(uniform_instance* instance, unsigned int maxNbEvaluations);
void uniform_keepSubtree(uniform_instance* instance);
unsigned int uniform_getMaxDepth(uniform_instance* instance);
void uniform_uninitInstance(uniform_instance** instance);

#endif
