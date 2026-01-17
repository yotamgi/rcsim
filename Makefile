PROJECT_NAME=rcsim

all: build-linux/${PROJECT_NAME} build-web/${PROJECT_NAME}.html

build-linux/Makefile: CMakeLists.txt
	cmake -B build-linux -DCMAKE_BUILD_TYPE=Release

build-web/Makefile: CMakeLists.txt
	emcmake cmake -B build-web -DPLATFORM=Web -DCMAKE_BUILD_TYPE=Debug

.PHONY:
build-linux/${PROJECT_NAME}: build-linux/Makefile
	make -C build-linux

.PHONY:
build-web/${PROJECT_NAME}.html: build-web/Makefile
	make -C build-web

.PHONY:
run_linux: build-linux/${PROJECT_NAME}
	make -C build-linux
	./build-linux/${PROJECT_NAME}

.PHONY:
run_web: build-web/${PROJECT_NAME}.html
	make -C build-web
	google-chrome http://0.0.0.0:8000/build-web/${PROJECT_NAME}.html
	python3 -m http.server

.PHONY:
clean:
	rm -rf build-linux build-web

