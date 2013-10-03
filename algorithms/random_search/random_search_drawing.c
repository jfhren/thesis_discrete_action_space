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

#include "random_search.h"

void random_search_drawingProcedure(SDL_Surface* screen, int screenWidth, int screenHeight, void* instance) {

    if(instance != NULL) {
        unsigned int i = 0;

        double rectangleWidth = (((screenWidth / 2.0) - 10) - (K * 5)) / (double)K;
        double heightRatio = (screenHeight - 25) * (1.0 - ((random_search_instance*)instance)->gamma);

        hlineRGBA(screen, (screenWidth / 2.0) + 5, screenWidth - 5, 5, 0, 0, 0, 255);

        for(; i < K; i++) {
            char str[255];

            double x1 = (screenWidth / 2.0) + 5 + ((rectangleWidth + 5) * i);
            double y1 = 5 + (heightRatio * (1.0 / (1.0 - ((random_search_instance*)instance)->gamma) - ((random_search_instance*)instance)->QValues[i]));
            double x2 = x1 + rectangleWidth;
            double y2 = screenHeight - 20;

            boxRGBA(screen, x1, y1, x2, y2, 0, 0, 0, 255);

            sprintf(str, "%u", i);
            stringRGBA(screen, (screenWidth / 2.0) + ((rectangleWidth + 5) * (i + 1)) - (rectangleWidth / 2.0), screenHeight-15, str, 0, 0, 0, 255);

        }
    }

}
