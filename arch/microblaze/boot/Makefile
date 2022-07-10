# SPDX-License-Identifier: GPL-2.0
#
# arch/microblaze/boot/Makefile
#

<<<<<<< HEAD
targets := linux.bin linux.bin.gz linux.bin.ub simpleImage.%
=======
targets := linux.bin linux.bin.gz linux.bin.ub simpleImage.*
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

OBJCOPYFLAGS := -R .note -R .comment -R .note.gnu.build-id -O binary

$(obj)/linux.bin: vmlinux FORCE
	$(call if_changed,objcopy)
<<<<<<< HEAD
	@echo 'Kernel: $@ is ready' ' (#'`cat .version`')'

$(obj)/linux.bin.ub: $(obj)/linux.bin FORCE
	$(call if_changed,uimage)
	@echo 'Kernel: $@ is ready' ' (#'`cat .version`')'

$(obj)/linux.bin.gz: $(obj)/linux.bin FORCE
	$(call if_changed,gzip)
	@echo 'Kernel: $@ is ready' ' (#'`cat .version`')'

quiet_cmd_cp = CP      $< $@$2
	cmd_cp = cat $< >$@$2 || (rm -f $@ && echo false)
=======

$(obj)/linux.bin.ub: $(obj)/linux.bin FORCE
	$(call if_changed,uimage)

$(obj)/linux.bin.gz: $(obj)/linux.bin FORCE
	$(call if_changed,gzip)
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

quiet_cmd_strip = STRIP   $< $@$2
	cmd_strip = $(STRIP) -K microblaze_start -K _end -K __log_buf \
				-K _fdt_start $< -o $@$2

UIMAGE_LOADADDR = $(CONFIG_KERNEL_BASE_ADDR)
<<<<<<< HEAD
UIMAGE_IN = $@
UIMAGE_OUT = $@.ub

$(obj)/simpleImage.%: vmlinux FORCE
	$(call if_changed,cp,.unstrip)
	$(call if_changed,objcopy)
	$(call if_changed,uimage)
	$(call if_changed,strip,.strip)
	@echo 'Kernel: $(UIMAGE_OUT) is ready' ' (#'`cat .version`')'

clean-files += simpleImage.*.unstrip linux.bin.ub
=======

$(obj)/simpleImage.$(DTB): vmlinux FORCE
	$(call if_changed,objcopy)

$(obj)/simpleImage.$(DTB).ub: $(obj)/simpleImage.$(DTB) FORCE
	$(call if_changed,uimage)

$(obj)/simpleImage.$(DTB).unstrip: vmlinux FORCE
	$(call if_changed,shipped)

$(obj)/simpleImage.$(DTB).strip: vmlinux FORCE
	$(call if_changed,strip)
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
