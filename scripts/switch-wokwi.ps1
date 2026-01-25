param(
  [Parameter(Mandatory = $true)]
  [ValidateSet('rack','room')]
  [string]$Mode
)

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $MyInvocation.MyCommand.Path | Split-Path -Parent

$files = if ($Mode -eq 'rack') {
  @{
    "diagram.json" = "diagram.rack.json";
    "wokwi.toml"  = "wokwi.rack.toml";
  }
} else {
  @{
    "diagram.json" = "diagram.room.json";
    "wokwi.toml"  = "wokwi.room.toml";
  }
}

foreach ($dest in $files.Keys) {
  $src = Join-Path $root $files[$dest]
  $dst = Join-Path $root $dest
  if (-not (Test-Path $src)) {
    throw "Missing source file: $src"
  }
  Copy-Item -Path $src -Destination $dst -Force
}

Write-Host "Active Wokwi set to $Mode"
Write-Host "- diagram.json <= $($files['diagram.json'])"
Write-Host "- wokwi.toml <= $($files['wokwi.toml'])"
