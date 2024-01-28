@echo off

set INTERMEDIATE=.\intermediate
set DEBUG_RUN_TREE=.\run_trees\debug
set EXE=engine.exe
set PDB=engine.pdb

set SOURCE=main.cpp lib\glad\src\glad.c lib\imgui\imgui*.cpp lib\imgui\examples\imgui_impl_glfw.cpp lib\imgui\examples\imgui_impl_opengl3.cpp 
set INCLUDE_DIRS= /I"lib\glad\include" /I"lib\glfw\include" /I"lib\imgui" /I"lib\imgui\examples" /I"lib\stb"

set DEBUG_MACROS=/DDEBUG
set DEBUG_COMPILE_FLAGS=/c /WX /W4 /Od /Zi /EHsc /Fo%INTERMEDIATE%\

REM mkdir %INTERMEDIATE%
REM mkdir %DEBUG_RUN_TREE%

cl %DEBUG_COMPILE_FLAGS% %DEBUG_MACROS% %INCLUDE_DIRS% %SOURCE%
if %errorlevel% neq 0 exit /b %errorlevel%

set DEBUG_LINK_FLAGS=/DEBUG:FULL /OUT:"%INTERMEDIATE%\%EXE%"

link %DEBUG_LINK_FLAGS% %INTERMEDIATE%\*.obj
if %errorlevel% neq 0 exit /b %errorlevel%

xcopy /Y %INTERMEDIATE%\%EXE% %DEBUG_RUN_TREE%
xcopy /Y %INTERMEDIATE%\%PDB% %DEBUG_RUN_TREE%

%DEBUG_RUN_TREE%\%EXE%

