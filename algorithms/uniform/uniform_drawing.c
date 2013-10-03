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

#ifndef USE_SDL
#error SDL should not be used!
#endif

#include <SDL/SDL.h>
#include <SDL/SDL_gfxPrimitives.h>

#include "uniform.h"

static void drawTree(SDL_Surface* screen, uniform_node* n, double start, double stop, unsigned int depth, double hSpaceTree) {

    if(n->children != NULL) {
        unsigned int i = 0, depthChild = depth + 1;
        double spaceBetween = (stop - start) / K;
        double startChild, stopChild;

        for(; i < K; i++) {
            startChild = start + (i * spaceBetween);
            stopChild = startChild + spaceBetween;
            drawTree(screen, n->children + i, startChild, stopChild, depthChild, hSpaceTree);
            aalineRGBA(screen, start + ((stop-start) / 2), depth * hSpaceTree, startChild + (spaceBetween / 2), depthChild * hSpaceTree , 0, 0, 0, 255);
        }
    }

}


void uniform_drawingProcedure(SDL_Surface* screen, int screenWidth, int screenHeight, void* instance) {

    static unsigned int maxViewedDepth = 0;
    static double hSpaceTree = 10.0;

    if(instance == NULL) {
        uniform_node* root = ((uniform_instance*)instance)->root;

        if(root != NULL) {
            unsigned int crtMaxDepth = uniform_getMaxDepth((uniform_instance*)instance);
            
            if(crtMaxDepth > maxViewedDepth) {
                maxViewedDepth = crtMaxDepth;
                hSpaceTree = screenHeight / maxViewedDepth;
                printf("maxViewedDepth: %u\n", maxViewedDepth);
            }
        }

        drawTree(screen, root, screenWidth / 2.0, screenWidth, 0, hSpaceTree);
    }

}
