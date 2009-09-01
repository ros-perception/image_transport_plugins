all: installed

VERSION		= 1.0
TARBALL     = build/libtheora-$(VERSION).tar.gz
TARBALL_URL = http://downloads.xiph.org/releases/theora/libtheora-$(VERSION).tar.bz2
UNPACK_CMD  = tar -xvjf
SOURCE_DIR  = build/libtheora-$(VERSION)

include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked
	echo "Building LibTheora..."
	cd $(SOURCE_DIR) && ./autogen.sh --prefix=$(PWD)/installed &&  make && make install
	touch installed

wiped:
	make wipe
	touch wiped

wipe:
	rm -rf build installed
