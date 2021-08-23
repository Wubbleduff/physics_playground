
libs = -lX11 lib/glfw/lib/libglfw3.a -lrt -lm -ldl
include_dirs = -I lib/glad/include -I lib/glfw/include 
source = lib/glad/src/glad.c \
	 src/main.cpp \
	 src/graphics.cpp \
	 src/game_math.cpp \
	 src/level.cpp \
	 src/rigid_body.cpp
executable = physics_playground

build_and_run:
	g++ `pkg-config --cflags glfw3` $(include_dirs) $(source) `pkg-config --static --libs glfw3` -g -o $(executable)
	./$(executable)

build:
	g++ `pkg-config --cflags glfw3` $(include_dirs) $(source) `pkg-config --static --libs glfw3` -g -o $(executable)

