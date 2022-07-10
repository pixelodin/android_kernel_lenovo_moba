# SPDX-License-Identifier: GPL-2.0
noarg:
	$(MAKE) -C ../

TEST_GEN_PROGS := hugetlb_vs_thp_test subpage_prot prot_sao segv_errors
<<<<<<< HEAD
=======
TEST_GEN_PROGS_EXTENDED := tlbie_test
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
TEST_GEN_FILES := tempfile

top_srcdir = ../../../../..
include ../../lib.mk

$(TEST_GEN_PROGS): ../harness.c

$(OUTPUT)/prot_sao: ../utils.c

$(OUTPUT)/tempfile:
	dd if=/dev/zero of=$@ bs=64k count=1

<<<<<<< HEAD
=======
$(OUTPUT)/tlbie_test: LDLIBS += -lpthread
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
