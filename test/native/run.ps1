param(
    [string] $Compiler = 'g++',
    [string] $LegacySource = ''
)

$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$buildDirectory = Join-Path $projectRoot '.pio/native-tests'
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null

$arguments = @('-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
    '-I', $PSScriptRoot, '-I', (Join-Path $projectRoot 'include'))
if ($LegacySource) {
    $resolvedSource = (Resolve-Path -LiteralPath $LegacySource).Path.Replace('\', '/')
    $reference = "namespace legacy {`n#include `"$resolvedSource`"`n}`n"
    [IO.File]::WriteAllText((Join-Path $buildDirectory 'legacy_reference.h'), $reference)
    $arguments += @('-DLEGACY_REFERENCE', '-I', $buildDirectory, '-Wno-unused-variable')
}
$arguments += Join-Path $PSScriptRoot 'test_main.cpp'
$arguments += Get-ChildItem (Join-Path $projectRoot 'src') -Filter '*.cpp' | ForEach-Object { $_.FullName }
$executable = Join-Path $buildDirectory 'controller-tests.exe'
$arguments += @('-o', $executable)

$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
$env:PATH = (Split-Path $compilerPath) + ';' + $env:PATH
& $compilerPath @arguments
if ($LASTEXITCODE -ne 0) { throw 'Native test compilation failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Native tests failed' }
