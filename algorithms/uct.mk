CC_OPTIONS := -O3
USE_SDL := 1
CC := gcc
FLAGS := -W -Wall $(CC_OPTIONS) -ansi -std=c99 -pedantic -Werror$(if $(USE_SDL), -DUSE_SDL)
LIBS := -lm$(if $(USE_SDL), -lSDL -lSDLmain /usr/lib/libSDL_gfx.so) -largtable2 -lgsl -lgslcblas
BIN_DIR := ../bin
OBJ_DIR := ../obj

all: $(addprefix $(BIN_DIR)/uct_,$(PROBLEMS)) $(OBJ_DIR)/uct_limited.o 

$(OBJ_DIR)/uct.o: uct/uct.c uct/uct.h
	$(CC) -c $(FLAGS) $< -o $@

$(OBJ_DIR)/uct_limited.o: uct/uct.c uct/uct.h
	$(CC) -c $(FLAGS) -DLIMITED_DEPTH $< -o $@

$(OBJ_DIR)/uct_drawing.o: uct/uct_drawing.c uct/uct_drawing.h uct/uct.h
	$(CC) -c $(FLAGS) $< -o $@

$(OBJ_DIR)/main_uct.o: uct/main_uct.c
	$(CC) -c $(FLAGS) $< -o $@

.SECONDEXPANSION:
$(BIN_DIR)/uct_%: $(OBJ_DIR)/uct.o $(OBJ_DIR)/main_uct.o $(OBJ_DIR)/$$*.o $(if $(USE_SDL),$(OBJ_DIR)/uct_drawing.o) $$(if $(USE_SDL),$(OBJ_DIR)/viewer_$$*.o)
	$(CC) $(FLAGS) $(LIBS) $^ -o $@
