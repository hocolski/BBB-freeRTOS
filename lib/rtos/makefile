
ROOT=../../

# Device definition (EVM not necessary)
#
DEVICE=am335x

#
# Include makefile definitions. Includes compilation, linker and archiving
# options.
#
include ${ROOT}/makedefs_ti

#
# Source files for given target
#
SOURCE=${RTOS_SRC}/*.c

# Rules for  building the source files
#
all: debug release

debug:
	mkdir -p debug
	$(CC) $(CFLAGS) -g $(SOURCE) 
	$(AR) $(ARFLAGS) debug/librtos.a *.o* 
	@mv *.o* debug/
	@mkdir -p ${RTOS_BIN}/debug
	@cp debug/librtos.a ${RTOS_BIN}/debug/librtos.a

release:
	mkdir -p release
	$(CC) $(CFLAGS) -g -O2 $(SOURCE) 
	$(AR) $(ARFLAGS) release/librtos.a *.o* 
	@mv *.o* release/
	@mkdir -p ${RTOS_BIN}/release
	@cp release/librtos.a ${RTOS_BIN}/release/librtos.a

clean: 
	@rm -rf debug release ${RTOS_BIN}/debug ${RTOS_BIN}/release


