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
#include <time.h>
#include <argtable2.h>
#include <math.h>
#include <string.h>

#include "../algorithms/optimistic/optimistic.h"

#ifdef BALL
#include "../problems/ball/ball.h"
#else
#ifdef CART_POLE
#include "../problems/cart_pole/cart_pole.h"
#else
#ifdef DOUBLE_CART_POLE
#include "../problems/double_cart_pole/double_cart_pole.h"
#else
#ifdef MOUNTAIN_CAR
#include "../problems/mountain_car/mountain_car.h"
#else
#ifdef ACROBOT
#include "../problems/acrobot/acrobot.h"
#else
#ifdef BOAT
#include "../problems/boat/boat.h"
#else
#ifdef CART_POLE_BINARY
#include "../problems/cart_pole_binary/cart_pole_binary.h"
#else
#ifdef SWIMMER
#include "../problems/swimmer/swimmer.h"
#endif
#endif
#endif
#endif
#endif
#endif
#endif
#endif

unsigned int* parseUnsignedIntList(char* str, unsigned int* nbItems) {

    unsigned int maxNbItems = 16;
    unsigned int* list = (unsigned int*)malloc(sizeof(unsigned int) * maxNbItems);

    unsigned int size = strlen(str);
    char* token = NULL;
    char* tmp = (char*)malloc(sizeof(char) * (size + 1));
    memcpy(tmp, str, sizeof(char) * (size + 1));

    *nbItems = 0;
    token = strtok(tmp, ",");
    while(token != NULL) {
        (*nbItems)++;

        if(*nbItems > maxNbItems) {
            maxNbItems += maxNbItems;
            list = realloc(list, sizeof(unsigned int) * maxNbItems);
        }

        list[(*nbItems - 1)] = strtol(token, NULL, 10);
        token = strtok(NULL, ",");        
    }

    list = realloc(list, sizeof(unsigned int) * *nbItems);
    free(tmp);

    return list;

}


int main(int argc, char* argv[]) {

#ifdef BALL
    double discountFactor = 0.9;
#else
#ifdef CART_POLE
    double discountFactor = 0.95;
#else
#ifdef DOUBLE_CART_POLE
    double discountFactor = 0.95;
#else
#ifdef MOUNTAIN_CAR
    double discountFactor = 0.99;
#else
#ifdef ACROBOT
    double discountFactor = 0.95;
#else
#ifdef BOAT
    double discountFactor = 0.95;
#else
#ifdef CART_POLE_BINARY
    double discountFactor = 0.95;
#else
#ifdef SWIMMER
    double discountFactor = 0.95;
#endif
#endif
#endif
#endif
#endif
#endif
#endif
#endif

    FILE* initFileFd = NULL;
    state** initialStates = NULL;
    FILE* results = NULL;
    char str[1024];
    unsigned int i = 0;
    unsigned int h = 0;
    unsigned int* ns = NULL;
    unsigned int nbN = 0;
    unsigned int n = 0;
    unsigned int nbSteps = 0;
    unsigned int timestamp = time(NULL);
    int readFscanf = -1;

    optimistic_instance* optimistic = NULL;

    struct arg_file* initFile = arg_file1(NULL, "init", "<file>", "File containing the inital state");
    struct arg_str* r = arg_str1("n", NULL, "<s>", "List of maximum numbers of evaluations");
    struct arg_int* s = arg_int1("s", NULL, "<n>", "Number of steps");
    struct arg_int* k = arg_int1("k", NULL, "<n>", "Branching factor of the problem");
    struct arg_file* where = arg_file1(NULL, "where", "<file>", "Directory where we save the outputs");
    struct arg_end* end = arg_end(6);

    int nerrors = 0;
    void* argtable[6];

    argtable[0] = initFile;
    argtable[1] = r;
    argtable[2] = s;
    argtable[3] = k;
    argtable[4] = where;
    argtable[5] = end;

    if(arg_nullcheck(argtable) != 0) {
        printf("error: insufficient memory\n");
        arg_freetable(argtable, 6);
        return EXIT_FAILURE;
    }

    nerrors = arg_parse(argc, argv, argtable);

    if(nerrors > 0) {
        printf("%s:", argv[0]);
        arg_print_syntax(stdout, argtable, "\n");
        arg_print_errors(stdout, end, argv[0]);
        arg_freetable(argtable, 6);
        return EXIT_FAILURE;
    }

    initGenerativeModelParameters();
    K = k->ival[0];
    initGenerativeModel();

    initFileFd = fopen(initFile->filename[0], "r");
    readFscanf = fscanf(initFileFd, "%u\n", &n);
    initialStates = (state**)malloc(sizeof(state*) * n);
    
    for(; i < n; i++) {
        readFscanf = fscanf(initFileFd, "%s\n", str);
        initialStates[i] = makeState(str);
    }
    fclose(initFileFd);

    nbSteps = s->ival[0];
    ns = parseUnsignedIntList((char*)r->sval[0], &nbN);

    optimistic = optimistic_initInstance(NULL, discountFactor);

    sprintf(str, "%s/%u_results_%u_%u.csv", where->filename[0], timestamp, K, nbSteps);
    results = fopen(str, "w");

    for(h = 0; h < nbN; h++) {
        double sumRewards = 0.0;

        for(i = 0; i < n; i++) {
            unsigned int j = 0;
            state* crt = copyState(initialStates[i]);

            optimistic_resetInstance(optimistic, crt);
            for(; j < nbSteps; j++) {
                char isTerminal = 0;
                double reward = 0.0;
                state* nextState = NULL;

                optimistic_keepSubtree(optimistic);
                action* optimalAction = optimistic_planning(optimistic, ns[h]);
                isTerminal = nextStateReward(crt, optimalAction, &nextState, &reward) < 0 ? 1 : 0;
                freeState(crt);
                crt = nextState;
                sumRewards += reward;
                if(isTerminal)
                    break;
            }
            optimistic_resetInstance(optimistic, crt);
            freeState(crt);

            printf(">>>>>>>>>>>>>> %uth initial state processed\n", i + 1);
            fflush(NULL);
        }

        fprintf(results, "%u,%.15f\n", ns[h], sumRewards / (double)n);
        printf(">>>>>>>>>>>>>> n = %u  done\n\n", ns[h]);
        fflush(NULL);
    }

    fclose(results);

    arg_freetable(argtable, 6);

    for(i = 0; i < n; i++)
        freeState(initialStates[i]);

    free(initialStates);

    optimistic_uninitInstance(&optimistic);

    freeGenerativeModel();
    freeGenerativeModelParameters();

    return EXIT_SUCCESS;

}
