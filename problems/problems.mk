CC_OPTIONS := -O3
USE_SDL := 1
CC := gcc
FLAGS := -W -Wall $(CC_OPTIONS) -ansi -std=c99 -pedantic -Werror
LIBS := -lm$(if $(USE_SDL), -lSDL -lSDLmain /usr/lib/libSDL_gfx.so) -largtable2
OBJ_DIR := ../obj

all: $(addsuffix .o,$(addprefix $(OBJ_DIR)/,$(PROBLEMS))$(if $(USE_SDL), $(addprefix $(OBJ_DIR)/viewer_,$(PROBLEMS))))

.SECONDEXPANSION:
$(OBJ_DIR)/viewer_%.o: $$*/viewer_$$*.c viewer.h
	$(CC) -c $(FLAGS) $< -o $@

$(OBJ_DIR)/%.o: $$*/$$*.c $$*/$$*.h generative_model.h
	$(CC) -c $(FLAGS) $< -o $@
