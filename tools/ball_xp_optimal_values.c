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
#include <argtable2.h>

#define LIMITED_DEPTH
#include "../algorithms/optimistic/optimistic.h"
#include "../problems/ball/ball.h"

int main(int argc, char* argv[]) {

    double discountFactor = 0.9;
    unsigned int i = 0;
    unsigned int n = 0;
    FILE* initFileFd = NULL;
    FILE* outputFileFd = NULL;
    unsigned int nbIterations = 0;
    int readFscanf = -1;

    optimistic_instance* optimistic = NULL;

    struct arg_file* initFile = arg_file1(NULL, "init", "<file>", "File containing the inital state");
    struct arg_int* k = arg_int1("k", NULL, "<n>", "The branching factor of the problem");
    struct arg_int* it = arg_int1("n", NULL, "<n>", "The number of iterations");
    struct arg_file* outputFile = arg_file1("o", NULL, "<file>", "The output file");
    struct arg_end* end = arg_end(5);

    void* argtable[5];

    int nerrors = 0;

    argtable[0] = initFile;
    argtable[1] = it;
    argtable[2] = outputFile;
    argtable[3] = k;
    argtable[4] = end;

    if(arg_nullcheck(argtable) != 0) {
        printf("error: insufficient memory\n");
        arg_freetable(argtable, 5);
        return EXIT_FAILURE;
    }

    nerrors = arg_parse(argc, argv, argtable);

    if(nerrors > 0) {
        printf("%s:", argv[0]);
        arg_print_syntax(stdout, argtable, "\n");
        arg_print_errors(stdout, end, argv[0]);
        arg_freetable(argtable, 5);
        return EXIT_FAILURE;
    }

    nbIterations = it->ival[0];

    initGenerativeModelParameters();
    K = k->ival[0];
    initGenerativeModel();

    outputFileFd = fopen(outputFile->filename[0], "w");

    initFileFd = fopen(initFile->filename[0], "r");
    readFscanf = fscanf(initFileFd, "%u\n", &n);

    arg_freetable(argtable, 5);

    optimistic = optimistic_initInstance(NULL, discountFactor);

    for(; i < n; i++) {
        char str[1024];
        unsigned int j = 0;

        readFscanf = fscanf(initFileFd, "%s\n", str);
        state* initial = makeState(str);
        double crtOptimalValue = 0.0;
        unsigned int crtOptimalAction = 0;

        for(; j < K; j++) {
            state* nextState = NULL;
            double reward = 0.0;
            char isTerminal = 0;

            isTerminal = nextStateReward(initial, actions[j], &nextState, &reward);
            optimistic_resetInstance(optimistic, nextState);
            freeState(nextState);
            optimistic_planning(optimistic, nbIterations);

            if(optimistic->crtOptimalValue > crtOptimalValue) {
                crtOptimalValue = optimistic->crtOptimalValue;
                crtOptimalAction = j;
            }

            fprintf(outputFileFd, "%.15f,", optimistic->crtOptimalValue);
            printf("%uth action done\n", j+1);
            fflush(outputFileFd);
        }
        fprintf(outputFileFd, "%u\n", crtOptimalAction);
        freeState(initial);
        printf("%uth initial state processed\n", i+1);
        fflush(outputFileFd);
    }

    fclose(outputFileFd);
    fclose(initFileFd);

    optimistic_uninitInstance(&optimistic);

    freeGenerativeModel();
    freeGenerativeModelParameters();

    return EXIT_SUCCESS;

}
