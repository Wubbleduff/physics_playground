@echo off

set INTERMEDIATE=.\intermediate
set DEBUG_RUN_TREE=.\run_trees\debug
set EXE=engine.exe
set PDB=engine.pdb

set SOURCE=lib\glad\src\glad.c src\*.cpp src\dynamics\*.cpp src\collision_detection\*.cpp src\collision_resolution\*.cpp src\sandbox\*.cpp
set INCLUDE_DIRS=/I"src" /I"lib\glad\include" /I"lib\glfw\include" /I"lib\imgui" /I"lib\stb"
set LIBS=user32.lib gdi32.lib shell32.lib opengl32.lib lib\glfw\glfw.lib

set DEBUG_MACROS=/DDEBUG
set DEBUG_COMPILE_FLAGS=/c /Zi /EHsc /Fo%INTERMEDIATE%\


mkdir %INTERMEDIATE%
mkdir %DEBUG_RUN_TREE%


cl %DEBUG_COMPILE_FLAGS% %DEBUG_MACROS% %INCLUDE_DIRS% %SOURCE%
if %errorlevel% neq 0 exit /b %errorlevel%



set DEBUG_LINK_FLAGS=/DEBUG:FULL /OUT:"%INTERMEDIATE%\%EXE%"

link %DEBUG_LINK_FLAGS% %LIBS% %INTERMEDIATE%\*.obj
if %errorlevel% neq 0 exit /b %errorlevel%



xcopy /Y %INTERMEDIATE%\%EXE% %DEBUG_RUN_TREE%
xcopy /Y %INTERMEDIATE%\%PDB% %DEBUG_RUN_TREE%
xcopy /Y /E assets %DEBUG_RUN_TREE%\assets\



%DEBUG_RUN_TREE%\%EXE%

