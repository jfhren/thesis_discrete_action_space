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

#ifndef UCT_H
#define UCT_H


#ifdef LIMITED_DEPTH
#define UCT_MAX_DEPTH 512
#else
#define UCT_MAX_DEPTH 32768
#endif

#include "../../problems/generative_model.h"

typedef struct uct_node_struct {
        state* s;                                   // The state associated with this node
        double reward;                              // The reward associated with the transition to this state
        double discountedSum;                       // The discounted sum of reward from the root to this state
        unsigned int depth;                         // The depth of this node within the tree
        unsigned int n;
        struct uct_node_struct* crtOptimalLeaf;     // The leaf with the biggest discounted sum of reward within this subtree
        struct uct_node_struct* crtNextOpennedLeaf; // The next leaf that should be openned within this subtree
        unsigned int trajectoryId;
		unsigned int id;                            // Index of the node in the children array of his father. Not useful for root node
		char isClosedBranch;                        // 1 if leaves from this node or the current leaf don't need to be openned later, 0 else
		struct uct_node_struct* father;             // Father of the node. NULL if node is the root
		struct uct_node_struct* children;           // Array of K children. NULL if node is a leaf
}	uct_node;

typedef struct {

        double gamma;
        uct_node* root;

        unsigned int crtNbEvaluations;
        unsigned int realNbEvaluations;
        unsigned int totalNbEvaluations;

        unsigned int crtOptimalAction;
        double crtOptimalValue;
        uct_node* crtOptimalLeaf;

        double gammaPowers[UCT_MAX_DEPTH];
        double bounds[UCT_MAX_DEPTH];

        uct_node* nextOpennedNode;

}   uct_instance;

uct_instance* uct_initInstance(state* initial, double discountFactor);
void uct_resetInstance(uct_instance* instance, state* initial);
action* uct_planning(uct_instance* instance, unsigned int maxNbEvaluations);
void uct_keepSubtree(uct_instance* instance);
unsigned int uct_getMaxDepth(uct_instance* instance);
void uct_uninitInstance(uct_instance** instance);

#endif
