param(
  [string]$ProjectRoot=(Join-Path $PSScriptRoot '..\firmware'),
  [string]$ToolchainBin='E:\STM32tool\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin',
  [string]$MakeExe='E:\STM32tool\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.100.202601091506\tools\bin\make.exe'
)
$ErrorActionPreference='Stop'
if(!(Test-Path -LiteralPath (Join-Path $ProjectRoot 'Debug\makefile'))){throw 'Import firmware into CubeIDE and Build Debug once to generate Debug/makefile.'}
if(!(Test-Path -LiteralPath (Join-Path $ToolchainBin 'arm-none-eabi-gcc.exe'))){throw 'Set -ToolchainBin to the ARM GCC bin directory.'}
if(!(Test-Path -LiteralPath $MakeExe)){throw 'Set -MakeExe to CubeIDE make.exe.'}
$savedPath=$env:Path
$env:Path="$ToolchainBin;$(Split-Path $MakeExe);$env:SystemRoot\System32;$env:SystemRoot"
Push-Location (Join-Path $ProjectRoot 'Debug')
try {
  $output=& $MakeExe 'SHELL=cmd.exe' -B -j8 all 2>&1
  $result=$LASTEXITCODE
  $output | Where-Object {$_ -match '(warning:|error:|undefined reference|Finished building|text\s+data|\sSTM32CubeIDE.elf\s*$)'}
  if($result -ne 0){$output;throw "Build exit code $result"}
  if($output -match 'warning:|error:'){throw 'Build contains warnings/errors'}
  # CubeIDE's generated makefile only emits ELF/list/size for this project.
  # Regenerate flashable images so an older BIN/HEX cannot survive a rebuild.
  $objcopy=Join-Path $ToolchainBin 'arm-none-eabi-objcopy.exe'
  & $objcopy -O binary STM32CubeIDE.elf STM32CubeIDE.bin
  if($LASTEXITCODE -ne 0){throw 'BIN generation failed'}
  & $objcopy -O ihex STM32CubeIDE.elf STM32CubeIDE.hex
  if($LASTEXITCODE -ne 0){throw 'HEX generation failed'}
  'FULL_BUILD_OK'
}finally{Pop-Location;$env:Path=$savedPath}
