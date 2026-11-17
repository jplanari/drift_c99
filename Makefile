# Build drift_c99.
#
# Dependències: només LibmCS (libm) més C99. Cap altra llibreria externa.
#   - Host (tests):  enllaça amb la libm de LibmCS compilada per al host.
#   - SPARC (vol):   CC=sparc-gaisler-elf-gcc (o BCC), LIBM=<libm.a creuada>.
#
# Exemple:
#   make LIBMCS_DIR=/ruta/a/libmcs test
#   make LIBMCS_DIR=/ruta/a/libmcs check-deps

CC     ?= gcc
CSTD   = -std=c99
WARN   = -Wall -Wextra -pedantic -Werror
# -fno-tree-loop-distribute-patterns: evita que gcc converteixi bucles de bytes
# (drift_zero/drift_copy) en crides a memset/memcpy de libc.
CFLAGS = $(CSTD) $(WARN) -O2 -fno-tree-loop-distribute-patterns -fno-stack-protector -Iinclude $(EXTRA)

LIBMCS_DIR   ?= /home/claude/libmcs_src/libmcs
LIBMCS_INC   = -I$(LIBMCS_DIR)/include
LIBM         ?= $(LIBMCS_DIR)/build-x86_64-linux-gnu/bin/libm.a

SRC = src/hs_detect.c src/hs_image.c src/kf4.c src/tracker.c src/drift_rng.c

# Símbols indefinits permesos a l'objecte: només funcions de libm.
ALLOWED_UNDEF = fabsf|fabs|sqrtf|sqrt|logf|log|cosf|cos

all: test_host

test_host: test/test_host.c $(SRC) $(LIBM)
	$(CC) $(CFLAGS) $(LIBMCS_INC) -o $@ test/test_host.c $(SRC) $(LIBM)

test: test_host
	./test_host

# Comparador amb el golden master (cal generar-lo abans amb golden/golden_gen.cpp).
# Ús: make golden-check GOLDEN_DIR=golden_out
GOLDEN_DIR ?= golden_out
golden_check: test/golden_check.c $(SRC) $(LIBM)
	$(CC) $(CFLAGS) $(LIBMCS_INC) -o $@ test/golden_check.c $(SRC) $(LIBM)

golden-check: golden_check
	./golden_check $(GOLDEN_DIR)

# Comprova que els objectes de src/ només necessiten símbols de libm.
check-deps: $(SRC)
	@fail=0; \
	for f in $(SRC); do \
	  $(CC) $(CFLAGS) $(LIBMCS_INC) -c $$f -o /tmp/drift_chk.o || exit 1; \
	  ext=$$(nm -u /tmp/drift_chk.o | awk '{print $$2}' | grep -Ev '^(drift_|hs_|kf4_|tracker_)' || true); \
	  for s in $$ext; do \
	    if echo "$$s" | grep -Eq '^($(ALLOWED_UNDEF))$$'; then \
	      echo "  $$f -> $$s (libm)"; \
	    else \
	      echo "  $$f -> $$s  NO PERMÈS"; fail=1; \
	    fi; \
	  done; \
	done; \
	rm -f /tmp/drift_chk.o; \
	if [ $$fail -eq 0 ]; then echo "OK: només dependències de libm"; else exit 1; fi

clean:
	rm -f test_host golden_check

.PHONY: all test check-deps clean
