PLATFORMS ?= linux/amd64,linux/arm64

all: image

image:
	docker build \
		--tag ghcr.io/cpslab-asu/ngc-rover-ha/controller:latest \
		--platform $(PLATFORMS) \
		.
