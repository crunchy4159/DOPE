# setup_imgui.ps1
# Run from DOPE-main project root OR from the scripts\ subfolder:
#   cd C:\path\to\DOPE-main
#   powershell -ExecutionPolicy Bypass -File scripts\setup_imgui.ps1

param(
    [string]$ImGuiVersion = "v1.91.8"
)

$ErrorActionPreference = "Stop"

# Resolve project root whether script is run from root or scripts\
$ProjectRoot = $PSScriptRoot
if ($ProjectRoot -match '\\scripts$') {
    $ProjectRoot = $ProjectRoot -replace '\\scripts$', ''
}
if (-not (Test-Path (Join-Path $ProjectRoot "platformio.ini"))) {
    if (Test-Path (Join-Path (Get-Location) "platformio.ini")) {
        $ProjectRoot = (Get-Location).Path
    } else {
        Write-Error "Cannot locate platformio.ini. Run from DOPE-main or its scripts\ folder."
        exit 1
    }
}

$ImGuiDir    = Join-Path $ProjectRoot "third_party\imgui"
$TmpZip      = Join-Path $env:TEMP "imgui_${ImGuiVersion}.zip"
$TmpExtract  = Join-Path $env:TEMP "imgui_extract"
$DownloadUrl = "https://github.com/ocornut/imgui/archive/refs/tags/${ImGuiVersion}.zip"

Write-Host ""
Write-Host "DOPE ImGui Setup" -ForegroundColor Cyan
Write-Host "================" -ForegroundColor Cyan
Write-Host "  Project root : $ProjectRoot"
Write-Host "  ImGui target : $ImGuiDir"
Write-Host "  Version      : $ImGuiVersion"
Write-Host ""

# Skip if already populated
if (Test-Path (Join-Path $ImGuiDir "imgui.h")) {
    Write-Host "imgui.h already present -- nothing to do." -ForegroundColor Green
    exit 0
}

# Download
Write-Host "Downloading ImGui $ImGuiVersion ..." -ForegroundColor Yellow
try {
    Invoke-WebRequest -Uri $DownloadUrl -OutFile $TmpZip -UseBasicParsing
} catch {
    Write-Error "Download failed: $_"
    exit 1
}
Write-Host "Download complete." -ForegroundColor Green

# Extract
Write-Host "Extracting ..." -ForegroundColor Yellow
if (Test-Path $TmpExtract) { Remove-Item $TmpExtract -Recurse -Force }
Expand-Archive -Path $TmpZip -DestinationPath $TmpExtract
$extracted = Get-ChildItem $TmpExtract -Directory | Select-Object -First 1
if (-not $extracted) {
    Write-Error "Unexpected archive layout."
    exit 1
}

# Copy core files
New-Item -ItemType Directory -Force -Path $ImGuiDir | Out-Null
$coreFiles = @(
    "imgui.h", "imgui.cpp", "imgui_internal.h",
    "imgui_draw.cpp", "imgui_tables.cpp", "imgui_widgets.cpp",
    "imconfig.h", "imstb_rectpack.h", "imstb_textedit.h", "imstb_truetype.h"
)
foreach ($f in $coreFiles) {
    $src = Join-Path $extracted.FullName $f
    if (Test-Path $src) { Copy-Item $src $ImGuiDir -Force }
    else { Write-Warning "Not found in archive: $f" }
}

# Copy backend files
$backendsDir = Join-Path $ImGuiDir "backends"
New-Item -ItemType Directory -Force -Path $backendsDir | Out-Null
$backendFiles = @(
    "imgui_impl_dx11.h", "imgui_impl_dx11.cpp",
    "imgui_impl_win32.h", "imgui_impl_win32.cpp"
)
foreach ($f in $backendFiles) {
    $src = Join-Path $extracted.FullName "backends\$f"
    if (Test-Path $src) { Copy-Item $src $backendsDir -Force }
    else { Write-Warning "Not found in archive: backends\$f" }
}

# Verify
$required = @(
    (Join-Path $ImGuiDir "imgui.h"),
    (Join-Path $ImGuiDir "imgui.cpp"),
    (Join-Path $ImGuiDir "imgui_draw.cpp"),
    (Join-Path $ImGuiDir "imgui_tables.cpp"),
    (Join-Path $ImGuiDir "imgui_widgets.cpp"),
    (Join-Path $backendsDir "imgui_impl_dx11.h"),
    (Join-Path $backendsDir "imgui_impl_dx11.cpp"),
    (Join-Path $backendsDir "imgui_impl_win32.h"),
    (Join-Path $backendsDir "imgui_impl_win32.cpp")
)
$ok = $true
foreach ($f in $required) {
    if (-not (Test-Path $f)) { Write-Warning "Missing: $f"; $ok = $false }
}

# Cleanup
Remove-Item $TmpZip     -Force -ErrorAction SilentlyContinue
Remove-Item $TmpExtract -Recurse -Force -ErrorAction SilentlyContinue

if ($ok) {
    Write-Host ""
    Write-Host "ImGui $ImGuiVersion installed successfully." -ForegroundColor Green
    Write-Host "You can now run:  scripts\run_native_gui.bat" -ForegroundColor Cyan
    Write-Host ""
} else {
    Write-Error "Some files are still missing. See warnings above."
    exit 1
}