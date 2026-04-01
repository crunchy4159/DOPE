# Update the local clone from origin. Usage: .\update_repo.ps1 [-Force]
param([switch]$Force)
Write-Output "Updating repository (Force=$Force)"
if ($Force) {
    git fetch --all --prune
    $branch = (git rev-parse --abbrev-ref HEAD).Trim()
    git reset --hard "origin/$branch"
    git clean -fdx
} else {
    git fetch --all --prune
    git pull --rebase
}
Write-Output "Done."
