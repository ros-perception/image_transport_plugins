all: installed

VERSION		= 1.0
TARBALL     = build/libtheora-$(VERSION).tar.bz2
TARBALL_URL = http://pr.willowgarage.com/downloads/libtheora-$(VERSION).tar.bz2
UNPACK_CMD  = tar -xvjf
SOURCE_DIR  = build/libtheora-$(VERSION)
MD5SUM_FILE = libtheora-$(VERSION).tar.bz2.md5sum

include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked
	echo "Building LibTheora..."
	cd $(SOURCE_DIR) && ./autogen.sh --prefix=$(CURDIR)/installed &&  make && make install
	touch -c installed

wiped:
	make wipe
	touch wiped

wipe:
	rm -rf build installed
