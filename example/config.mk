# rbphys examples build configuration

RAYLIB = ${HOME}/.local
RAYLIB_INC = -I${RAYLIB}/include
RAYLIB_LIB = -L${RAYLIB}/lib64 -lraylib

RBPHYS = -I../

OTHER_LIB = -lrt -lm -ldl

INCS = ${RAYLIB_INC} ${RBPHYS}
LIBS = -pthread ${RAYLIB_LIB} ${OTHER_LIB}

# change to -Os if not debugging
DEBUG = -ggdb

CFLAGS = -std=c99 -pedantic -Wall -Wno-deprecated-declarations ${DEBUG} ${INCS} ${CPPFLAGS}
LDFLAGS = ${DEBUG} ${LIBS}
CC = cc
