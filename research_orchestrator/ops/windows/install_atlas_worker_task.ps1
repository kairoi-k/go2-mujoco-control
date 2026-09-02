$ErrorActionPreference = "Stop"

$taskName = "Go2_Atlas_Research_Worker"
$templatePath = Join-Path $PSScriptRoot "Go2_Atlas_Research_Worker.xml"
$template = Get-Content -LiteralPath $templatePath -Raw -Encoding UTF8
$identity = [System.Security.Principal.WindowsIdentity]::GetCurrent()
$userSid = $identity.User.Value
$userName = $identity.Name
$xml = $template.Replace("__ATLAS_USER_SID__", $userSid).Replace("__ATLAS_USER_NAME__", $userName).Replace('encoding="UTF-8"', 'encoding="UTF-16"')
$tempPath = Join-Path ([System.IO.Path]::GetTempPath()) ("Go2_Atlas_Research_Worker.{0}.xml" -f ([guid]::NewGuid()))

try {
    [System.IO.File]::WriteAllText($tempPath, $xml, [System.Text.Encoding]::Unicode)
    & schtasks.exe /Create /TN $taskName /XML $tempPath /F
    if ($LASTEXITCODE -ne 0) {
        throw "schtasks registration failed with exit code $LASTEXITCODE"
    }
    & schtasks.exe /Run /TN $taskName
    if ($LASTEXITCODE -ne 0) {
        throw "schtasks start failed with exit code $LASTEXITCODE"
    }
    & schtasks.exe /Query /TN $taskName /FO LIST /V
} finally {
    Remove-Item -LiteralPath $tempPath -Force -ErrorAction SilentlyContinue
}
