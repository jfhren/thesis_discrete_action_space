CC_OPTIONS := -O3
USE_SDL := 1
CC := gcc
FLAGS := -W -Wall $(CC_OPTIONS) -ansi -std=c99 -pedantic -Werror$(if $(USE_SDL), -DUSE_SDL)
LIBS := -lm$(if $(USE_SDL), -lSDL -lSDLmain /usr/lib/libSDL_gfx.so) -largtable2 -lgsl -lgslcblas
BIN_DIR := ../bin
OBJ_DIR := ../obj

all: $(addprefix $(BIN_DIR)/random_search_, $(PROBLEMS)) $(OBJ_DIR)/random_search_limited.o

$(OBJ_DIR)/random_search.o: random_search/random_search.c random_search/random_search.h
	$(CC) -c $(FLAGS) $< -o $@

$(OBJ_DIR)/random_search_limited.o: random_search/random_search.c random_search/random_search.h
	$(CC) -c $(FLAGS) -DLIMITED_DEPTH $< -o $@

$(OBJ_DIR)/random_search_drawing.o: random_search/random_search_drawing.c random_search/random_search_drawing.h random_search/random_search.h
	$(CC) -c $(FLAGS) $< -o $@

$(OBJ_DIR)/main_random_search.o: random_search/main_random_search.c
	$(CC) -c $(FLAGS) $< -o $@

.SECONDEXPANSION:
$(BIN_DIR)/random_search_%: $(OBJ_DIR)/random_search.o $(OBJ_DIR)/main_random_search.o $(OBJ_DIR)/$$*.o $(if $(USE_SDL),$(OBJ_DIR)/random_search_drawing.o) $$(if $(USE_SDL),$(OBJ_DIR)/viewer_$$*.o)
	$(CC) $(FLAGS) $(LIBS) $^ -o $@
