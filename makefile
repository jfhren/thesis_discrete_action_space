#Uncomment to build with debug symbols
#export CC_OPTIONS := -g
#Uncomment to build without SDL (and thus without viewer)
#export USE_SDL := 

#The list of problems found in the problems directory
export PROBLEMS := $(shell ls -d problems/*/ | cut -f 2 -d '/')
BIN_DIR := ./bin
OBJ_DIR := ./obj

.PHONY: make_directories clean 

all: make_directories
	$(MAKE) -C problems -f problems.mk -e
	$(MAKE) -C algorithms -f optimistic.mk -e
	$(MAKE) -C algorithms -f uniform.mk -e
	$(MAKE) -C algorithms -f random_search.mk -e
	$(MAKE) -C algorithms -f uct.mk -e

tools: all
	$(MAKE) -C tools -f tools.mk -e

make_directories:
	mkdir -p $(BIN_DIR)
	mkdir -p $(OBJ_DIR)

clean:
	rm -rf $(BIN_DIR)
	rm -rf $(OBJ_DIR)

