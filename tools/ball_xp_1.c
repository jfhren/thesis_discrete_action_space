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

#include "../problems/ball/ball.h"

int main(int argc, char* argv[]) {

    double discountFactor = 0.9;

    FILE* initFileFd = NULL;
    /*FILE** optimisticFd = NULL;
    FILE** randomFd = NULL;
    FILE** uctFd = NULL;
    FILE** uniformFd = NULL;*/
    FILE** combinedFd = NULL;
    FILE* optimalFd = NULL;

    optimistic_instance* optimistic = NULL;
    random_search_instance* random_search = NULL;
    uct_instance* uct = NULL;
    uniform_instance* uniform = NULL;

    unsigned int maxDepth = 0;
    unsigned int i = 0;
    unsigned int n = 0;
    state** initialStates = NULL;
    unsigned int timestamp = time(NULL);

    struct arg_file* initFile = arg_file1(NULL, "init", "<file>", "File containing the inital state");
    struct arg_int* d = arg_int1("d", NULL, "<n>", "Maximum depth of an uniform tree which the number of call per step");
    struct arg_int* k = arg_int1("k", NULL, "<n>", "Branching factor of the problem");
    struct arg_file* where = arg_file1(NULL, "where", "<file>", "Directory where we save the outputs");
    struct arg_file* optimal = arg_file1(NULL, "optimal", "<file>", "File containing the optimal values");
    struct arg_end* end = arg_end(6);

    void* argtable[6];
    int nerrors = 0;

    argtable[0] = initFile;
    argtable[1] = where;
    argtable[2] = d;
    argtable[3] = k;
    argtable[4] = optimal;
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

    optimalFd = fopen(optimal->filename[0], "r");
    initFileFd = fopen(initFile->filename[0], "r");
    fscanf(initFileFd, "%u\n", &n);
    initialStates = (state**)malloc(sizeof(state*) * n);
    
    for(i = 0; i < n; i++) {
        char str[1024];
        fscanf(initFileFd, "%s\n", str);
        initialStates[i] = makeState(str);
    }

    maxDepth = d->ival[0];

    /*optimisticFd = (FILE**)malloc(sizeof(FILE*) * maxDepth);
    randomFd = (FILE**)malloc(sizeof(FILE*) * maxDepth);
    uctFd = (FILE**)malloc(sizeof(FILE*) * maxDepth);
    uniformFd = (FILE**)malloc(sizeof(FILE*) * maxDepth);*/
    combinedFd = (FILE**)malloc(sizeof(FILE*) * maxDepth);

    for(i = 1; i <= maxDepth; i++) {
        char str[1024];
/*        sprintf(str, "%s/%u_optimistic_%u_%u.csv", where->filename[0], timestamp, K, i);
        optimisticFd[i - 1] = fopen(str, "w");
        sprintf(str, "%s/%u_random_search_%u_%u.csv", where->filename[0], timestamp, K, i);
        randomFd[i - 1] = fopen(str, "w");
        sprintf(str, "%s/%u_uct_%u_%u.csv", where->filename[0], timestamp, K, i);
        uctFd[i - 1] = fopen(str, "w");
        sprintf(str, "%s/%u_uniform_%u_%u.csv", where->filename[0], timestamp, K, i);
        uniformFd[i - 1] = fopen(str, "w");*/
        sprintf(str, "%s/%u_combined_%u_%u.csv", where->filename[0], timestamp, K, i);
        combinedFd[i - 1] = fopen(str, "w");
        fprintf(combinedFd[i - 1], "optimal,n,optimistic,random search,uct,uniform\n");
    }

    arg_freetable(argtable, 6);

    optimistic = optimistic_initInstance(initialStates[0], discountFactor);
    random_search = random_search_initInstance(initialStates[0], discountFactor);
    uct = uct_initInstance(initialStates[0], discountFactor);
    uniform = uniform_initInstance(initialStates[0], discountFactor);

    for(i = 0; i < n; i++) {
        unsigned int j = 1;
        unsigned int maxNbIterations = K;
        char str[255];
        fscanf(optimalFd, "%s\n", str);

        for(; j <= maxDepth; j++) {
            optimistic_planning(optimistic, maxNbIterations);
            //fprintf(optimisticFd[j - 1], "%.15f\n", optimistic->crtOptimalValue);
            fprintf(combinedFd[j - 1], "%s,%u,", str,maxNbIterations);
            fprintf(combinedFd[j - 1], "%.15f,", optimistic->crtOptimalValue);
            maxNbIterations += pow(K, j+1);
        }

        optimistic_resetInstance(optimistic, initialStates[((i + 1) < n) ? (i + 1) : i]);

        printf("optimistic: %uth initial state processed\n", i+1);

        for(j = 1; j <= maxDepth; j++) {
            random_search_planning(random_search, maxNbIterations);
            //fprintf(randomFd[j - 1], "%.15f\n", random_search->crtOptimalValue);
            fprintf(combinedFd[j - 1], "%.15f,", random_search->crtOptimalValue);
            maxNbIterations += pow(K, j+1);
        }

        random_search_resetInstance(random_search, initialStates[((i + 1) < n) ? (i + 1) : i]);

        printf("random_search: %uth initial state processed\n", i+1);

        for(j = 1; j <= maxDepth; j++) {
            uct_planning(uct, maxNbIterations);
            //fprintf(uctFd[j - 1], "%.15f\n", uct->crtOptimalValue);
            fprintf(combinedFd[j - 1], "%.15f,", uct->crtOptimalValue);
            maxNbIterations += pow(K, j+1);
        }

        uct_resetInstance(uct, initialStates[((i + 1) < n) ? (i + 1) : i]);

        printf("uct: %uth initial state processed\n", i+1);

        for(j = 1; j <= maxDepth; j++) {
            uniform_planning(uniform, maxNbIterations);
            //fprintf(uniformFd[j - 1], "%.15f\n", uniform->root->crtOptimalLeaf->discountedSum);
            fprintf(combinedFd[j - 1], "%.15f\n", uniform->root->crtOptimalLeaf->discountedSum);
            maxNbIterations += pow(K, j+1);
        }

        uniform_resetInstance(uniform, initialStates[((i + 1) < n) ? (i + 1) : i]);

        printf("uniform: %uth initial state processed\n", i+1);

        printf("%uth initial state processed\n", i+1);

    }

    for(i = 0; i < maxDepth; i++) {
        /*fclose(optimisticFd[i]);
        fclose(randomFd[i]);
        fclose(uctFd[i]);
        fclose(uniformFd[i]);*/
        fclose(combinedFd[i]);
    }

    for(i = 0; i < n; i++)
        freeState(initialStates[i]);

    free(initialStates);

    /*free(optimisticFd);
    free(randomFd);
    free(uctFd);
    free(uniformFd);*/
    free(combinedFd);

    optimistic_uninitInstance(&optimistic);
    random_search_uninitInstance(&random_search);
    uct_uninitInstance(&uct);
    uniform_uninitInstance(&uniform);

    freeGenerativeModel();
    freeGenerativeModelParameters();

    return EXIT_SUCCESS;

}
