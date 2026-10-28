@echo off
rem Build the f32 VALU random vector exporter with MSVC (Windows local runs).
rem   rtl\gpu\valu\vec\build_dump.bat [build_dir]      (default build\valu_fp_dump under the repo root)
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
set "HERE=%~dp0"
set "ROOT=%HERE%..\..\..\.."
set "BD=%~1"
if "%BD%"=="" set "BD=%ROOT%\build\valu_fp_dump"
cmake -S "%HERE%." -B "%BD%" -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release || exit /b 1
cmake --build "%BD%" || exit /b 1
echo VALU_FP_DUMP_DONE
