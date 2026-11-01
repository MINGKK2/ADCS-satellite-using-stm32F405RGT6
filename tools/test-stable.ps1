param(
  [string]$ProjectRoot=(Join-Path $PSScriptRoot '..\firmware'),
  [string]$Compiler='C:\msys64\ucrt64\bin\gcc.exe',
  [string]$Python='C:\msys64\ucrt64\bin\python.exe',
  [string]$Node='C:\Users\Admin\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe'
)
$ErrorActionPreference='Stop'
$repoRoot=Split-Path -Parent $PSScriptRoot
$tests=Join-Path $repoRoot 'tests'
$viewer=Join-Path $repoRoot 'viewer'
if(!(Test-Path -LiteralPath $Compiler)){$Compiler=(Get-Command gcc -ErrorAction Stop).Source}
if(!(Test-Path -LiteralPath $Python)){$Python=(Get-Command python -ErrorAction Stop).Source}
if(!(Test-Path -LiteralPath $Node)){$Node=(Get-Command node -ErrorAction Stop).Source}
$env:ADCS_HOST_CC=$Compiler
function Invoke-Checked([string]$exe,[string[]]$arguments){& $exe @arguments;if($LASTEXITCODE -ne 0){throw "$exe failed: $LASTEXITCODE"}}
$cflags=@('-std=c11','-O2','-Wall','-Wextra','-Werror')
$includes=@("-I$tests","-I$ProjectRoot\Core\Inc")
foreach($name in @('parser','protocol','uart')){
  $src=switch($name){parser{'command_parser.c'}protocol{'telemetry_protocol.c'}uart{'telemetry_uart.c'}}
  $output=Join-Path $tests "test-$name.exe"
  Invoke-Checked $Compiler ($cflags+$includes+@((Join-Path $tests "test-$name.c"),(Join-Path $ProjectRoot "Core\Src\$src"),'-o',$output))
  Invoke-Checked $output @()
}
$output=Join-Path $tests 'test-calibration-rollback.exe'
Invoke-Checked $Compiler ($cflags+@('-DCALIBRATION_STORE_HOST_TEST',"-I$tests\flash-stub")+$includes+@((Join-Path $tests 'test-calibration-rollback.c'),(Join-Path $ProjectRoot 'Core\Src\calibration_store.c'),'-o',$output))
Invoke-Checked $output @()
Invoke-Checked $Python @((Join-Path $tests 'test-adcs-source.py'),$ProjectRoot)
Invoke-Checked $Node @((Join-Path $viewer 'test-dashboard.cjs'))
Invoke-Checked $Node @((Join-Path $viewer 'test-attitude-model.mjs'))
$elf=Join-Path $ProjectRoot 'Debug\STM32CubeIDE.elf'
if(!(Test-Path -LiteralPath $elf)){$elf=Join-Path $repoRoot 'artifacts\manual-pid\STM32CubeIDE.elf'}
if(!(Test-Path -LiteralPath $elf)){$elf=Join-Path $repoRoot 'artifacts\stable-debug\STM32CubeIDE.elf'}
Invoke-Checked $Python @((Join-Path $PSScriptRoot 'check-calibration-elf.py'),$elf)
'ALL_STABLE_TESTS_OK'
