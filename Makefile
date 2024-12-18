BUILD_EXAMPLES=OFF
BUILD_TYPE=Release
CMAKE_ARGS:=$(CMAKE_ARGS)

default:
	@mkdir build
	@cd build && cmake .. -DBUILD_EXAMPLES=$(BUILD_EXAMPLES) -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) $(CMAKE_ARGS) && make

debug:
	@make default BUILD_TYPE=Debug

apps:
	@make default BUILD_EXAMPLES=ON

debug_apps:
	@make debug BUILD_EXAMPLES=ON

clean:
	@rm -rf build*
