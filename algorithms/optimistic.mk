CC_OPTIONS := -O3
USE_SDL := 1
CC := gcc
FLAGS := -W -Wall $(CC_OPTIONS) -ansi -std=c99 -pedantic -Werror$(if $(USE_SDL), -DUSE_SDL)
LIBS := -lm$(if $(USE_SDL), -lSDL -lSDLmain /usr/lib/libSDL_gfx.so) -largtable2 -lgsl -lgslcblas
BIN_DIR := ../bin
OBJ_DIR := ../obj

all: $(addprefix $(BIN_DIR)/optimistic_,$(PROBLEMS)) $(OBJ_DIR)/optimistic_limited.o

$(OBJ_DIR)/optimistic.o: optimistic/optimistic.c optimistic/optimistic.h
	$(CC) -c $(FLAGS) $< -o $@

$(OBJ_DIR)/optimistic_limited.o: optimistic/optimistic.c optimistic/optimistic.h
	$(CC) -c $(FLAGS) -DLIMITED_DEPTH $< -o $@

$(OBJ_DIR)/optimistic_drawing.o: optimistic/optimistic_drawing.c optimistic/optimistic_drawing.h optimistic/optimistic.h
	$(CC) -c $(FLAGS) $< -o $@

$(OBJ_DIR)/main_optimistic.o: optimistic/main_optimistic.c
	$(CC) -c $(FLAGS) $< -o $@

.SECONDEXPANSION:
$(BIN_DIR)/optimistic_%: $(OBJ_DIR)/optimistic.o $(OBJ_DIR)/main_optimistic.o $(OBJ_DIR)/$$*.o $(if $(USE_SDL),$(OBJ_DIR)/optimistic_drawing.o) $$(if $(USE_SDL),$(OBJ_DIR)/viewer_$$*.o)
	$(CC) $(FLAGS) $(LIBS) $^ -o $@
