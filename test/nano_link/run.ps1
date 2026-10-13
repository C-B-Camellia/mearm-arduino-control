param([string] $Compiler='C:/msys64/ucrt64/bin/g++.exe')
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$build=Join-Path $root '.pio/nano-link-tests'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$env:PATH=(Split-Path $Compiler)+';'+$env:PATH
$flags=@('-std=c++11','-O2','-Wall','-Wextra','-Werror','-DMEARM_NANO_LINK','-DMEARM_BUTTON_RX=10','-DMEARM_BUTTON_BAUD=9600','-I',(Join-Path $root 'include'))
$driver=Join-Path $build 'd10-uart.exe'
& $Compiler @flags '-D__AVR__' '-I' (Join-Path $PSScriptRoot 'register_mock') (Join-Path $PSScriptRoot 'pin_uart_tests.cpp') '-o' $driver
if($LASTEXITCODE -ne 0) { throw 'D10 driver model compile failed' }
& $driver
if($LASTEXITCODE -ne 0) { throw 'D10 driver model tests failed' }
$integration=Join-Path $build 'uno-link.exe'
$argsList=$flags+@('-DMEARM_NATIVE_TEST','-I',(Join-Path $root 'test/native'),(Join-Path $root 'test/native/nano_link_tests.cpp'))
$argsList+=Get-ChildItem (Join-Path $root 'src') -Filter '*.cpp' | ForEach-Object { $_.FullName }
& $Compiler @argsList '-o' $integration
if($LASTEXITCODE -ne 0) { throw 'Nano protocol integration compile failed' }
& $integration
if($LASTEXITCODE -ne 0) { throw 'Nano protocol integration tests failed' }
