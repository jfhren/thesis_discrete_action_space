CC_OPTIONS := -O3
USE_SDL := 1
CC := gcc
FLAGS := -W -Wall $(CC_OPTIONS) -ansi -std=c99 -pedantic -Werror$(if $(USE_SDL), -DUSE_SDL)
LIBS := -lm$(if $(USE_SDL), -lSDL -lSDLmain /usr/lib/libSDL_gfx.so) -largtable2 -lgsl -lgslcblas
BIN_DIR := ../bin
OBJ_DIR := ../obj

all: $(addprefix $(BIN_DIR)/uniform_,$(PROBLEMS)) $(OBJ_DIR)/uniform_limited.o

$(OBJ_DIR)/uniform.o: uniform/uniform.c uniform/uniform.h
	$(CC) -c $(FLAGS) $< -o $@

$(OBJ_DIR)/uniform_limited.o: uniform/uniform.c uniform/uniform.h
	$(CC) -c $(FLAGS) -DLIMITED_DEPTH $< -o $@

$(OBJ_DIR)/uniform_drawing.o: uniform/uniform_drawing.c uniform/uniform_drawing.h uniform/uniform.h
	$(CC) -c $(FLAGS) $< -o $@

$(OBJ_DIR)/main_uniform.o: uniform/main_uniform.c
	$(CC) -c $(FLAGS) $< -o $@

.SECONDEXPANSION:
$(BIN_DIR)/uniform_%: $(OBJ_DIR)/uniform.o $(OBJ_DIR)/main_uniform.o $(OBJ_DIR)/$$*.o $(if $(USE_SDL),$(OBJ_DIR)/uniform_drawing.o) $$(if $(USE_SDL),$(OBJ_DIR)/viewer_$$*.o)
	$(CC) $(FLAGS) $(LIBS) $^ -o $@
