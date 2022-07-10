# SPDX-License-Identifier: GPL-2.0
<<<<<<< HEAD
TEST_PROGS := ptrace-gpr ptrace-tm-gpr ptrace-tm-spd-gpr \
=======
TEST_GEN_PROGS := ptrace-gpr ptrace-tm-gpr ptrace-tm-spd-gpr \
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
              ptrace-tar ptrace-tm-tar ptrace-tm-spd-tar ptrace-vsx ptrace-tm-vsx \
              ptrace-tm-spd-vsx ptrace-tm-spr ptrace-hwbreak ptrace-pkey core-pkey \
              perf-hwbreak

top_srcdir = ../../../../..
include ../../lib.mk

<<<<<<< HEAD
all: $(TEST_PROGS)

CFLAGS += -m64 -I../../../../../usr/include -I../tm -mhtm -fno-pie

ptrace-pkey core-pkey: child.h
ptrace-pkey core-pkey: LDLIBS += -pthread

$(TEST_PROGS): ../harness.c ../utils.c ../lib/reg.S ptrace.h

clean:
	rm -f $(TEST_PROGS) *.o
=======
CFLAGS += -m64 -I../../../../../usr/include -I../tm -mhtm -fno-pie

$(OUTPUT)/ptrace-pkey $(OUTPUT)/core-pkey: child.h
$(OUTPUT)/ptrace-pkey $(OUTPUT)/core-pkey: LDLIBS += -pthread

$(TEST_GEN_PROGS): ../harness.c ../utils.c ../lib/reg.S ptrace.h
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
