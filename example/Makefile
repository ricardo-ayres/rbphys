include config.mk

SRC = const_rotation.c top.c orbit.c collide.c
OBJ = ${SRC:.c=.o}
BIN = ${SRC:.c=}

all: options ${BIN}

options:
	@echo build flags:
	@echo "CFLAGS   = ${CFLAGS}"
	@echo "LDFLAGS  = ${LDFLAGS}"
	@echo "CC       = ${CC}"

.c.o:
	@echo CC $<
	@${CC} -c ${CFLAGS} $<

${OBJ}: config.mk ../rbphys.h

${BIN}: ${OBJ}
	@echo ${CC} -o $@
	@${CC} -o $@ $@.o ${LDFLAGS}

clean:
	@echo cleaning
	@rm -f ${BIN} ${OBJ}

.PHONY: all options clean
