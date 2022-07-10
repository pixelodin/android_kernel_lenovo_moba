# SPDX-License-Identifier: GPL-2.0
<<<<<<< HEAD
TEST_PROGS := signal signal_tm

all: $(TEST_PROGS)

$(TEST_PROGS): ../harness.c ../utils.c signal.S

CFLAGS += -maltivec
signal_tm: CFLAGS += -mhtm
=======
TEST_GEN_PROGS := signal signal_tm

CFLAGS += -maltivec
$(OUTPUT)/signal_tm: CFLAGS += -mhtm
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

top_srcdir = ../../../../..
include ../../lib.mk

<<<<<<< HEAD
clean:
	rm -f $(TEST_PROGS) *.o
=======
$(TEST_GEN_PROGS): ../harness.c ../utils.c signal.S
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
