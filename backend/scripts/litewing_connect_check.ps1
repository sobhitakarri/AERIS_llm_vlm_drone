# Joins the LiteWing Wi-Fi, confirms the drone answers, then runs the link check, all in one go
# so Windows cannot switch back to another network in between.
#   powershell -ExecutionPolicy Bypass -File backend\scripts\litewing_connect_check.ps1
param([string]$Ssid = "LiteWing_68EE8FCF62CD", [string]$Drone = "192.168.43.42")

Write-Host "Joining $Ssid ..."
netsh wlan connect name=$Ssid | Out-Null
$ok = $false
$cur = ""
for ($i = 0; $i -lt 20; $i++) {
    Start-Sleep -Seconds 1
    $m = netsh wlan show interfaces | Select-String "^\s+SSID\s+:\s+(.+)$"
    if ($m) { $cur = $m.Matches[0].Groups[1].Value.Trim() }
    if ($cur -eq $Ssid) {
        $r = ping.exe -n 1 -w 1000 $Drone | Select-String "TTL="
        if ($r) { $ok = $true; break }
    }
}
if (-not $ok) {
    Write-Host "Could not reach $Drone on $Ssid (currently on '$cur'). Is the drone powered? Is the profile saved?"
    exit 1
}
Write-Host "On $Ssid, drone answers ping. Running the link check ..."
py -3.13 "$PSScriptRoot\litewing_link_check.py" "udp://$Drone"
