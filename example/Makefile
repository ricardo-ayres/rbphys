include config.mk

SRC = const_rotation.c
OBJ = ${SRC:.c=.o}

all: options const_rotation

options:
	@echo build flags:
	@echo "CFLAGS   = ${CFLAGS}"
	@echo "LDFLAGS  = ${LDFLAGS}"
	@echo "CC       = ${CC}"

.c.o:
	@echo CC $<
	@${CC} -c ${CFLAGS} $<

${OBJ}: config.mk

const_rotation: ${OBJ}
	@echo ${CC} -o $@
	@${CC} -o $@ ${OBJ} ${LDFLAGS}

clean:
	@echo cleaning
	@rm -f flatspin ${OBJ}

.PHONY: all options clean
