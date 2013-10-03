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

#ifndef OPTIMISTIC_H
#define OPTIMISTIC_H

#ifdef LIMITED_DEPTH
    #define OPTIMISTIC_MAX_DEPTH 512
#else
    #define OPTIMISTIC_MAX_DEPTH 32768
#endif

#include "../../problems/generative_model.h"

typedef struct {
		double bound;				// Bound on a leaf.
		double discountedSum;		// Discounted sum to a leaf.
		unsigned int depth;			// Depth of a leaf.
}	optimistic_node_values;

typedef struct optimistic_node_struct {
        state* s;                           // The state associated with this node
        double reward;                      // The reward associated with the transition to this state
		void* values;						// If is a node, point to the leaf containing the max bound else point to the value of the leaf.
		unsigned int trajectoryId;			// Index of the child containing the max bounded leaf.
		unsigned int id;					// Index of the node in the children array of his father. Not useful for root node.
		char isClosedBranch;				// 1 if leaves from this node or the current leaf don't need to be openned later, 0 else.
		struct optimistic_node_struct* father;					// Father of the node. NULL if node is the root.
		struct optimistic_node_struct* children;				// Array of K children. NULL if node is a leaf.
}	optimistic_node;

typedef struct {

        double gamma;
        optimistic_node* root;

        unsigned int crtNbEvaluations;
        unsigned int realNbEvaluations;
        unsigned int totalNbEvaluations;
        unsigned int crtOptimalAction;

        double crtOptimalValue;

        optimistic_node* crtOptimalLeaf;

        double gammaPowers[OPTIMISTIC_MAX_DEPTH];
        double bounds[OPTIMISTIC_MAX_DEPTH];

        optimistic_node* nextOpennedNode;

}   optimistic_instance;

optimistic_instance* optimistic_initInstance(state* initial, double discountFactor);
void optimistic_resetInstance(optimistic_instance* instance, state* initial);
action* optimistic_planning(optimistic_instance* instance, unsigned int maxNbEvaluations);
void optimistic_keepSubtree(optimistic_instance* instance);
unsigned int optimistic_getMaxDepth(optimistic_instance* instance);
void optimistic_uninitInstance(optimistic_instance** instance);

#endif
