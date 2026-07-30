# make-sums.ps1
# Generates GNU-sha256sum-compatible manifests for the ZeroCyclePaper data folder.
#
#   SHA256SUMS.txt    hashes of the per-day .zip bundles as uploaded
#   CONTENTS.sha256   (-Deep only) hashes of the .BIN entries inside each zip
#
# Run:
#   powershell -ExecutionPolicy Bypass -File make-sums.ps1
#   powershell -ExecutionPolicy Bypass -File make-sums.ps1 -Deep

param(
    [string] $Root    = "C:\Users\gmiek\OneDrive\!!ZeroCyclePaper",
    [string] $Pattern = "*.zip",
    [switch] $Deep
)

if (-not (Test-Path -LiteralPath $Root)) {
    Write-Error "Root not found: $Root"
    exit 1
}

$utf8NoBom = New-Object System.Text.UTF8Encoding($false)

function Write-Manifest {
    param([string[]] $Lines, [string] $Path)
    [System.IO.File]::WriteAllText($Path, ($Lines -join "`n") + "`n", $utf8NoBom)
}

# ---- 1. hash the archives themselves -------------------------------------

$files = Get-ChildItem -LiteralPath $Root -Recurse -File -Filter $Pattern |
         Sort-Object FullName

if (-not $files) {
    Write-Error "No files matching '$Pattern' under $Root"
    exit 1
}

Write-Host "Hashing $($files.Count) archive(s)..."

$lines = foreach ($f in $files) {
    $hash = (Get-FileHash -LiteralPath $f.FullName -Algorithm SHA256).Hash.ToLower()
    $rel  = $f.FullName.Substring($Root.Length + 1) -replace '\\', '/'
    Write-Host "  $rel"
    "$hash  $rel"
}

$out = Join-Path $Root "SHA256SUMS.txt"
Write-Manifest -Lines $lines -Path $out
Write-Host "`nWrote $($files.Count) entries to $out"

# ---- 2. optionally hash the entries inside each archive ------------------

if ($Deep) {
    Add-Type -AssemblyName System.IO.Compression.FileSystem

    Write-Host "`nDeep pass: hashing archive contents..."
    $deepLines = @()
    $sha = [System.Security.Cryptography.SHA256]::Create()

    foreach ($f in $files) {
        $relZip = $f.FullName.Substring($Root.Length + 1) -replace '\\', '/'
        $zip = [System.IO.Compression.ZipFile]::OpenRead($f.FullName)
        try {
            foreach ($entry in ($zip.Entries | Sort-Object FullName)) {
                if ($entry.FullName.EndsWith('/')) { continue }   # skip directories
                $stream = $entry.Open()
                try {
                    $bytes = $sha.ComputeHash($stream)
                } finally {
                    $stream.Dispose()
                }
                $hex = [System.BitConverter]::ToString($bytes).Replace('-', '').ToLower()
                $name = "$relZip!$($entry.FullName)"
                Write-Host "  $name"
                $deepLines += "$hex  $name"
            }
        } finally {
            $zip.Dispose()
        }
    }

    $sha.Dispose()
    $deepOut = Join-Path $Root "CONTENTS.sha256"
    Write-Manifest -Lines $deepLines -Path $deepOut
    Write-Host "`nWrote $($deepLines.Count) entries to $deepOut"
}

Write-Host "`nCopy the manifest(s) into the spinerebel repo and commit."
