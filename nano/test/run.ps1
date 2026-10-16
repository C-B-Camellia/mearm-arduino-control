param([string] $Compiler = 'C:/msys64/ucrt64/bin/g++.exe')
$ErrorActionPreference='Stop'
$nanoRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$build=Join-Path $nanoRoot '.pio/native-tests'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$env:PATH=(Split-Path $Compiler)+';'+$env:PATH
foreach($oled in @($false,$true)) {
    $argsList=@('-std=c++11','-O2','-Wall','-Wextra','-Werror','-I',$PSScriptRoot,'-I',(Join-Path $nanoRoot 'include'))
    if($oled) { $argsList+=@('-DMEARM_NANO_OLED','-DMEARM_OLED_ADDRESS=0x3C') }
    $binary=Join-Path $build ('nano-'+$oled+'.exe')
    $argsList+=@((Join-Path $PSScriptRoot 'nano_tests.cpp'),'-o',$binary)
    & $Compiler @argsList
    if($LASTEXITCODE -ne 0) { throw 'Nano test compilation failed' }
    & $binary (Join-Path $build ('panel-wire-'+$oled+'.bin'))
    if($LASTEXITCODE -ne 0) { throw 'Nano firmware tests failed' }
}
