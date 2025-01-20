PLATFORMS = linux/amd64,linux/arm64

all: image

image:
	make -C controller PLATFORMS=$(PLATFORMS) image
