

PROJECTS=`ls -d [012]*`
.SILENT:

default: build

help:
	@echo "Use one of the options: build clean zip"
	@exit 0

all: build

clean:
	@echo "Cleaning ..."
	for i in $(PROJECTS); do if [ -d "$$i" ]; then echo "Cleaning $$i ..." ; rm -f $$i.zip  ; ( cd $$i;  make clean ); fi; done

zip: clean
	@echo "Zipping ..."
	@for i in $(PROJECTS); do if [ -d "$$i" ]; then echo "Zipping $$i ..." ; zip -r $$i.zip $$i ; fi; done

build:
	@echo "Building ..."
	@for i in $(PROJECTS); do if [ -d "$$i" ]; then echo "Building $$i ..." ; ( cd $$i;  make build ); fi; done
