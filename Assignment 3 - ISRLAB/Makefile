.PHONY: build stop clean purge containers

DIRS := $(wildcard */.)
IMAGEDIRS := $(patsubst %/.,%,$(DIRS))

build: $(IMAGEDIRS)

$(IMAGEDIRS):
	@docker build -t app/$@_image ./$@

stop:
	@for dir in $(IMAGEDIRS); do \
		docker stop $$(docker ps -a -q --filter ancestor=app/$dir\_image); \
	done

containers:
	@for dir in $(IMAGEDIRS); do \
		docker ps -a -q --filter ancestor=app/$dir\_image; \
	done

clean:
	@for dir in $(IMAGEDIRS); do \
		docker rmi -f app/$$dir\_image; \
	done

purge:
	@for dir in $(IMAGEDIRS); do \
		docker stop $$(docker ps -a -q --filter ancestor=app/$$dir\_image); \
		docker rm $$(docker ps -a -q --filter ancestor=app/$$dir\_image); \
		docker rmi -f app/$$dir\_image; \
	done

print-dirs:
	@echo $(IMAGEDIRS)

.PHONY: $(IMAGEDIRS)

# .SILENT: $(IMAGEDIRS)