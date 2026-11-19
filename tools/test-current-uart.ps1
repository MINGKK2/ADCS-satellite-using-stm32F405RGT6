param(
  [string]$Compiler='C:\msys64\ucrt64\bin\gcc.exe'
)
$ErrorActionPreference='Stop'
if(!(Test-Path -LiteralPath $Compiler)){$Compiler=(Get-Command gcc -ErrorAction Stop).Source}
$root=Split-Path -Parent $PSScriptRoot
$tests=Join-Path $root 'tests'
$firmware=Join-Path $root 'firmware'
$temporary=Join-Path ([System.IO.Path]::GetTempPath()) ('adcs-uart-'+[guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $temporary | Out-Null
$common=@('-std=c11','-O2','-Wall','-Wextra','-Werror',"-I$tests","-I$firmware\Core\Inc")
function Run-Checked([string]$exe,[string[]]$parameters){
  & $exe @parameters
  if($LASTEXITCODE -ne 0){throw "$exe failed with exit $LASTEXITCODE"}
}
foreach($mode in @('little','big')) {
  $defines=if($mode -eq 'big'){@('-DTELEMETRY_PAYLOAD_BIG_ENDIAN=1','-DTELEMETRY_SENSOR_MSG_ID=0x0882')}else{@()}
  $exe=Join-Path $temporary "cfe-$mode.exe"
  Run-Checked $Compiler ($common+$defines+@("$tests\test-cfe-protocol.c","$firmware\Core\Src\telemetry_protocol.c",'-o',$exe))
  Run-Checked $exe @()
}
$exe=Join-Path $temporary 'uart.exe'
Run-Checked $Compiler ($common+@("$tests\test-uart.c","$firmware\Core\Src\telemetry_uart.c",'-o',$exe))
Run-Checked $exe @()
$exe=Join-Path $temporary 'calibration.exe'
Run-Checked $Compiler (@('-DCALIBRATION_STORE_HOST_TEST',"-I$tests\flash-stub")+$common+@("$tests\test-calibration-rollback.c","$firmware\Core\Src\calibration_store.c",'-o',$exe))
Run-Checked $exe @()
'CURRENT_UART_TESTS_OK'
Write-Output "Test executables: $temporary"
