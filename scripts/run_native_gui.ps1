param(
    [switch]$NoBuild
)

$ErrorActionPreference = "Stop"

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $repoRoot

$script:RunnerType = $null
$script:RunnerPath = $null
$venvPython = Join-Path $repoRoot ".venv\Scripts\python.exe"

function Test-PlatformIoModule {
    param([string]$PythonCmd)

    try {
        & $PythonCmd -m platformio --version *> $null
        return ($LASTEXITCODE -eq 0)
    } catch {
        return $false
    }
}

if (-not $script:RunnerType -and (Test-Path $venvPython -PathType Leaf)) {
    if (Test-PlatformIoModule -PythonCmd $venvPython) {
        $script:RunnerType = "venv-python"
        $script:RunnerPath = $venvPython
    }
}

if (-not $script:RunnerType -and (Get-Command pio -ErrorAction SilentlyContinue)) {
    $script:RunnerType = "pio"
    $script:RunnerPath = "pio"
}

if (-not $script:RunnerType -and (Get-Command platformio -ErrorAction SilentlyContinue)) {
    $script:RunnerType = "platformio"
    $script:RunnerPath = "platformio"
}

if (-not $script:RunnerType -and (Get-Command py -ErrorAction SilentlyContinue)) {
    if (Test-PlatformIoModule -PythonCmd "py") {
        $script:RunnerType = "py-module"
        $script:RunnerPath = "py"
    }
}

if (-not $script:RunnerType -and (Get-Command python -ErrorAction SilentlyContinue)) {
    if (Test-PlatformIoModule -PythonCmd "python") {
        $script:RunnerType = "python-module"
        $script:RunnerPath = "python"
    }
}

function Invoke-PlatformIoBuild {
    switch ($script:RunnerType) {
        "venv-python" {
            & $script:RunnerPath -m platformio run -e native_gui
        }
        "pio" {
            & pio run -e native_gui
        }
        "platformio" {
            & platformio run -e native_gui
        }
        "py-module" {
            & $script:RunnerPath -m platformio run -e native_gui
        }
        "python-module" {
            & $script:RunnerPath -m platformio run -e native_gui
        }
        default {
            throw "No PlatformIO runner available."
        }
    }
}

if (-not $script:RunnerType) {
    Write-Host "PlatformIO was not found on PATH or as a Python module." -ForegroundColor Red
    Write-Host ""
    Write-Host "Install options:" -ForegroundColor Yellow
    Write-Host "  1) py -m pip install -U platformio"
    Write-Host "  2) pip install -U platformio"
    Write-Host ""
    Write-Host "Then rerun: .\\scripts\\run_native_gui.ps1"
    exit 1
}

if ($script:RunnerPath) {
    Write-Host "Using PlatformIO runner: $script:RunnerType ($script:RunnerPath)"
} else {
    Write-Host "Using PlatformIO runner: $script:RunnerType"
}

if (-not $NoBuild) {
    Write-Host "Building native_gui..." -ForegroundColor Cyan
    Invoke-PlatformIoBuild
    $code = [int]$LASTEXITCODE
    if ($code -ne 0) {
        Write-Host "Build failed with exit code $code" -ForegroundColor Red
        exit $code
    }
}

$exe = Join-Path $repoRoot ".pio\build\native_gui\program.exe"
if (-not (Test-Path $exe)) {
    Write-Host "GUI executable not found at: $exe" -ForegroundColor Red
    Write-Host "Try building first: pio run -e native_gui"
    exit 1
}

Write-Host "Launching GUI..." -ForegroundColor Green
Start-Process -FilePath $exe -WorkingDirectory $repoRoot
