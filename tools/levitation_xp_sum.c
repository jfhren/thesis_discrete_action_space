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

#include "../algorithms/optimistic/optimistic.h"
#include "../algorithms/random_search/random_search.h"
#include "../algorithms/uct/uct.h"
#include "../algorithms/uniform/uniform.h"

#include "../problems/levitation/levitation.h"

int main(int argc, char* argv[]) {

    double discountFactor = 0.9;

    FILE* initFileFd = NULL;
    double* setPoints = NULL;
    unsigned int nbSetPoints = 0;
    unsigned int maxDepth = 0;
    FILE* results = NULL;
    char str[1024];
    unsigned int i = 0;
    unsigned int minDepth = 1;
    unsigned int crtDepth = 0;
    unsigned int maxNbIterations = 0;
    unsigned int nbSteps = 0;
    unsigned int timestamp = time(NULL);
    int readFscanf = -1;

    optimistic_instance* optimistic = NULL;
    random_search_instance* random_search = NULL;
    uct_instance* uct = NULL;
    uniform_instance* uniform = NULL;

    struct arg_file* initFile = arg_file1(NULL, "init", "<file>", "File containing the set points");
    struct arg_int* d2 = arg_int0(NULL, "min", "<n>", "Minimum depth to start from (min>0)");
    struct arg_int* d = arg_int1("d", NULL, "<n>", "Maximum depth of an uniform tree which the number of call per step");
    struct arg_int* s = arg_int1("s", NULL, "<n>", "Number of steps");
    struct arg_int* k = arg_int1("k", NULL, "<n>", "Branching factor of the problem");
    struct arg_file* where = arg_file1(NULL, "where", "<file>", "Directory where we save the outputs");
    struct arg_end* end = arg_end(7);

    int nerrors = 0;
    void* argtable[7];

    argtable[0] = initFile;
    argtable[1] = d2;
    argtable[2] = d;
    argtable[3] = s;
    argtable[4] = k;
    argtable[5] = where;
    argtable[6] = end;

    if(arg_nullcheck(argtable) != 0) {
        printf("error: insufficient memory\n");
        arg_freetable(argtable, 7);
        return EXIT_FAILURE;
    }

    nerrors = arg_parse(argc, argv, argtable);

    if(nerrors > 0) {
        printf("%s:", argv[0]);
        arg_print_syntax(stdout, argtable, "\n");
        arg_print_errors(stdout, end, argv[0]);
        arg_freetable(argtable, 7);
        return EXIT_FAILURE;
    }

    initGenerativeModelParameters();
    K = k->ival[0];
    initGenerativeModel();

    initFileFd = fopen(initFile->filename[0], "r");
    readFscanf = fscanf(initFileFd, "%u\n", &nbSetPoints);
    setPoints = (double*)malloc(sizeof(double) * nbSetPoints);
    
    for(; i < nbSetPoints; i++) {
        readFscanf = fscanf(initFileFd, "%s\n", str);
        setPoints[i] = strtod(str, NULL);
    }
    fclose(initFileFd);

    if(d2->count)
        minDepth = d2->ival[0];
    maxDepth = d->ival[0];
    maxNbIterations = K;
    nbSteps = s->ival[0];

    optimistic = optimistic_initInstance(NULL, discountFactor);
    random_search = random_search_initInstance(NULL, discountFactor);
    uct = uct_initInstance(NULL, discountFactor);
    uniform = uniform_initInstance(NULL, discountFactor);

    sprintf(str, "%s/%u_results_%u_%u.csv", where->filename[0], timestamp, K, nbSteps);
    results = fopen(str, "w");

    for(crtDepth = 1; crtDepth < minDepth; crtDepth++)
        maxNbIterations += pow(K, crtDepth+1);

    for(crtDepth = minDepth; crtDepth <= maxDepth; crtDepth++) {
        double averages[4] = {0.0, 0.0, 0.0, 0.0};
        state* crt1 = initState();
        state* crt2 = copyState(crt1);
        state* crt3 = copyState(crt1);
        state* crt4 = copyState(crt1);

        optimistic_resetInstance(optimistic, crt1);
        uct_resetInstance(uct, crt3);
        uniform_resetInstance(uniform, crt4);
        for(i = 0; i < nbSetPoints; i++) {
            unsigned int j = 0;

            parameters[10] = setPoints[i];

            for(; j < nbSteps; j++) {
                char isTerminal = 0;
                double reward = 0.0;
                state* nextState = NULL;

                optimistic_keepSubtree(optimistic);
                action* optimalAction = optimistic_planning(optimistic, maxNbIterations);
                isTerminal = nextStateReward(crt1, optimalAction, &nextState, &reward);
                freeState(crt1);
                crt1 = nextState;
                averages[0] += reward;
                if(isTerminal < 0)
                    break;
            }
            optimistic_resetInstance(optimistic, crt1);

            printf("Optimistic   : %uth set point processed\n", i + 1);

            for(j = 0; j < nbSteps; j++) {
                char isTerminal = 0;
                double reward = 0.0;
                state* nextState = NULL;

                random_search_resetInstance(random_search, crt2);
                action* optimalAction = random_search_planning(random_search, maxNbIterations);
                isTerminal = nextStateReward(crt2, optimalAction, &nextState, &reward);
                freeState(crt2);
                crt2 = nextState;
                averages[1] += reward;
                if(isTerminal < 0)
                    break;
            }
            random_search_resetInstance(random_search, crt1);

            printf("Random search: %uth set point processed\n", i + 1);

            for(j = 0; j < nbSteps; j++) {
                char isTerminal = 0;
                double reward = 0.0;
                state* nextState = NULL;

                uct_keepSubtree(uct);
                action* optimalAction = uct_planning(uct, maxNbIterations);
                isTerminal = nextStateReward(crt3, optimalAction, &nextState, &reward);
                freeState(crt3);
                crt3 = nextState;
                averages[2] += reward;
                if(isTerminal < 0)
                    break;
            }
            uct_resetInstance(uct, crt3);

            printf("Uct          : %uth set point processed\n", i + 1);

            for(j = 0; j < nbSteps; j++) {
                char isTerminal = 0;
                double reward = 0.0;
                state* nextState = NULL;

                uniform_keepSubtree(uniform);
                action* optimalAction = uniform_planning(uniform, maxNbIterations);
                isTerminal = nextStateReward(crt4, optimalAction, &nextState, &reward);
                freeState(crt4);
                crt4 = nextState;
                averages[3] += reward;
                if(isTerminal < 0)
                    break;
            }
            uniform_resetInstance(uniform, crt4);

            printf("Uniform      : %uth set point processed\n", i + 1);
            printf(">>>>>>>>>>>>>> %uth set point processed\n", i + 1);

        }

        fprintf(results, "%u,%.15f,%.15f,%.15f,%.15f\n", maxNbIterations, averages[0] / (double)nbSetPoints, averages[1] / (double)nbSetPoints, averages[2] / (double)nbSetPoints, averages[3] / (double)nbSetPoints);
        fflush(results);
        freeState(crt1);
        freeState(crt2);
        freeState(crt3);
        freeState(crt4);

        printf(">>>>>>>>>>>>>> %u depth done\n\n", crtDepth);

        maxNbIterations += pow(K, crtDepth+1);

    }

    fclose(results);

    arg_freetable(argtable, 7);

    free(setPoints);

    optimistic_uninitInstance(&optimistic);
    random_search_uninitInstance(&random_search);
    uct_uninitInstance(&uct);
    uniform_uninitInstance(&uniform);

    freeGenerativeModel();
    freeGenerativeModelParameters();

    return EXIT_SUCCESS;

}
