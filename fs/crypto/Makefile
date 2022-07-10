obj-$(CONFIG_FS_ENCRYPTION)	+= fscrypto.o

<<<<<<< HEAD
ccflags-y += -Ifs/ext4
ccflags-y += -Ifs/f2fs

fscrypto-y := crypto.o fname.o hooks.o keyinfo.o policy.o
fscrypto-$(CONFIG_BLOCK) += bio.o
fscrypto-$(CONFIG_PFK) += fscrypt_ice.o
=======
fscrypto-y := crypto.o \
	      fname.o \
	      hkdf.o \
	      hooks.o \
	      keyring.o \
	      keysetup.o \
	      keysetup_v1.o \
	      policy.o

fscrypto-$(CONFIG_BLOCK) += bio.o
fscrypto-$(CONFIG_FS_ENCRYPTION_INLINE_CRYPT) += inline_crypt.o
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
