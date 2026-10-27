# Opens a new terminal window with an SSH session to the Raspberry Pi.
$PiUser = "pi"
$PiHost = "192.168.5.1"
$PiPassword = "8AphJtr?"

$plink = Get-Command plink.exe -ErrorAction SilentlyContinue

if ($plink) {
    # PuTTY's plink accepts the password directly, so no prompt is needed
    Start-Process powershell -ArgumentList "-NoExit", "-Command", "& '$($plink.Source)' -ssh $PiUser@$PiHost -pw '$PiPassword'"
}
else {
    # Windows OpenSSH cannot take a password as an argument:
    # copy it to the clipboard so it can be pasted (right-click) at the prompt
    Set-Clipboard -Value $PiPassword
    Write-Host "Password copied to clipboard - right-click to paste it at the prompt."
    Start-Process powershell -ArgumentList "-NoExit", "-Command", "ssh $PiUser@$PiHost"
}
