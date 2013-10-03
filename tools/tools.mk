CC_OPTIONS := -O3
USE_SDL := 1
CC := gcc
FLAGS := -W -Wall $(CC_OPTIONS) -ansi -std=c99 -pedantic -Werror
LIBS := -lm$(if $(USE_SDL), -lSDL -lSDLmain /usr/lib/libSDL_gfx.so) -largtable2 -lgsl -lgslcblas
BIN_DIR := ../bin
OBJ_DIR := ../obj

all: $(addprefix $(BIN_DIR)/xp_sum_,$(PROBLEMS)) $(addprefix $(BIN_DIR)/xp_optimistic_sum_,$(PROBLEMS)) $(BIN_DIR)/xp_regret_ball $(BIN_DIR)/xp_optimal_values_ball $(BIN_DIR)/xp_initial_states_problems

$(BIN_DIR)/xp_regret_ball: $(OBJ_DIR)/xp_regret_ball.o $(OBJ_DIR)/optimistic_limited.o $(OBJ_DIR)/random_search_limited.o $(OBJ_DIR)/uct_limited.o $(OBJ_DIR)/uniform_limited.o $(OBJ_DIR)/ball.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@
	
$(BIN_DIR)/xp_optimal_values_ball: $(OBJ_DIR)/xp_optimal_values_ball.o $(OBJ_DIR)/optimistic_limited.o $(OBJ_DIR)/ball.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

$(BIN_DIR)/xp_initial_states_problems: $(OBJ_DIR)/xp_initial_states_problems.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

$(OBJ_DIR)/xp_regret_ball.o: ball_xp_regret.c
	$(CC) -c $(FLAGS) $< -o $@

$(OBJ_DIR)/xp_optimal_values_ball.o: ball_xp_optimal_values.c
	$(CC) -c $(FLAGS) $< -o $@

$(OBJ_DIR)/xp_initial_states_problems.o: problems_xp_initial_states.c
	$(CC) -c $(FLAGS) $< -o $@

$(OBJ_DIR)/xp_sum_%.o: problems_xp_sum.c
	$(CC) -c $(FLAGS) -D$(shell echo $* | tr a-z A-Z) $< -o $@

$(OBJ_DIR)/xp_optimistic_sum_%.o: problems_xp_sum_optimistic.c
	$(CC) -c $(FLAGS) -D$(shell echo $* | tr a-z A-Z) $< -o $@

$(OBJ_DIR)/xp_sum_levitation.o: levitation_xp_sum.c
	$(CC) -c $(FLAGS) $< -o $@

$(OBJ_DIR)/xp_optimistic_sum_levitation.o: levitation_xp_sum_optimistic.c
	$(CC) -c $(FLAGS) $< -o $@

.SECONDEXPANSION:
$(BIN_DIR)/xp_sum_%: $(OBJ_DIR)/xp_sum_$$*.o $(OBJ_DIR)/optimistic.o $(OBJ_DIR)/random_search.o $(OBJ_DIR)/uct.o $(OBJ_DIR)/uniform.o $(OBJ_DIR)/$$*.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

$(BIN_DIR)/xp_optimistic_sum_%: $(OBJ_DIR)/xp_optimistic_sum_$$*.o $(OBJ_DIR)/optimistic.o $(OBJ_DIR)/$$*.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@
