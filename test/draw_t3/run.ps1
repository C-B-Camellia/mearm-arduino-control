param([string] $Compiler='C:/msys64/ucrt64/bin/g++.exe',[string] $Suite='speed_tests.cpp',[string[]] $ExtraFlags=@())
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$build=Join-Path $root '.pio/draw-t3-audit'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$env:PATH=(Split-Path $Compiler)+';'+$env:PATH
$arguments=@('-std=c++11','-O2','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-DMEARM_NATIVE_TEST','-I',(Join-Path $root 'test/native'),'-I',(Join-Path $root 'include'),(Join-Path $PSScriptRoot $Suite))
$arguments+=Get-ChildItem (Join-Path $root 'src') -Filter '*.cpp' | Where-Object Name -ne 'motion.cpp' | ForEach-Object FullName
$arguments += $ExtraFlags
$exe=Join-Path $build 'draw-t3.exe'
& $Compiler @arguments '-o' $exe
if($LASTEXITCODE -ne 0) { throw 'DRAW T3 diagnostic compilation failed' }
& $exe
if($LASTEXITCODE -ne 0) { throw 'DRAW T3 diagnostic failed' }