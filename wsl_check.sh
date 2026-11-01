#!/bin/bash
# WSL-geheugen begrenzen (opgeroepen door start.sh, enkel onder WSL).
#
# Zonder %USERPROFILE%\.wslconfig mag WSL de helft van het RAM nemen en houdt
# het de bestandscache vast (vmmemWSL): na een `--pull` van de image (~5 GB)
# blijft er te weinig over voor Windows, dat dan begint te swappen (laptop
# traag, schijf op 100 %). Gemeten 31-10/1-11: container ~0,3-0,5 GB, en daarnaast
# in Windows de browser (Lichtblick + simulator) ~1 GB: 3 GB voor WSL volstaat bij 8 GB RAM.
#
# Bestaat .wslconfig niet: voorstel met een limiet volgens het RAM, en na een
# "j" aanmaken. Een bestaand bestand wordt nooit aangepast, enkel een tip.

# WSL_CHECK_TEST=1: testen buiten WSL (met nagebootste cmd.exe, wslpath, powershell.exe)
[ -n "$WSL_CHECK_TEST" ] || grep -qi microsoft /proc/version 2>/dev/null || exit 0

# Windows-gebruikersmap (cmd.exe vanuit een Windows-map: geen UNC-waarschuwing)
WINPROFILE=$(cd /mnt/c 2>/dev/null; cmd.exe /c 'echo %USERPROFILE%' 2>/dev/null | tr -d '\r')
[ -n "$WINPROFILE" ] || exit 0
CFG=$(wslpath "$WINPROFILE" 2>/dev/null)/.wslconfig
[ -d "$(dirname "$CFG")" ] || exit 0

if [ -f "$CFG" ]; then
    if ! grep -qiE '^\s*memory\s*=' "$CFG" || ! grep -qiE '^\s*autoMemoryReclaim\s*=' "$CFG"; then
        echo "Tip: $WINPROFILE\\.wslconfig heeft geen 'memory=' of 'autoMemoryReclaim='."
        echo "     Zonder die regels kan WSL te veel geheugen vasthouden (laptop traag). Zie de GitBook, Setup laptop."
    fi
    # Mirrored (vorig jaar voor DDS-multicast) is met Zenoh niet meer nodig; deelt de poorten
    # met Windows (8080/8765/9090/7447 al bezet -> Lichtblick of bridge start niet)
    if grep -qiE '^\s*networkingMode\s*=\s*mirrored' "$CFG"; then
        echo "Tip: $WINPROFILE\\.wslconfig gebruikt networkingMode=mirrored. Dat is niet meer nodig."
        echo "     Problemen met Lichtblick of de robot? Verwijder die regel, dan in PowerShell: wsl --shutdown"
    fi
    exit 0
fi

# RAM van de laptop (Windows); anders schatten: WSL krijgt standaard de helft
RAM_GB=$(cd /mnt/c 2>/dev/null; powershell.exe -NoProfile -Command \
    '[math]::Round((Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory / 1GB)' 2>/dev/null | tr -d '\r')
[[ $RAM_GB =~ ^[0-9]+$ ]] || RAM_GB=$(( $(awk '/^MemTotal/{print $2}' /proc/meminfo) * 2 / 1048576 ))
if   [ "$RAM_GB" -le 8 ];  then MEM=3GB
elif [ "$RAM_GB" -le 16 ]; then MEM=6GB
else                            MEM=8GB
fi

echo "WSL heeft nog geen geheugenlimiet (laptop: ~${RAM_GB} GB RAM). Voorstel voor $WINPROFILE\\.wslconfig:"
echo "    memory=$MEM, swap=2GB, autoMemoryReclaim=gradual, sparseVhd=true"
ANSWER=n
if [ -t 0 ]; then
    read -r -p "Nu aanmaken? [J/n] " ANSWER
    ANSWER=${ANSWER:-j}
fi
case $ANSWER in
    j|J|y|Y)
        printf '%s\r\n' \
            "# Aangemaakt door turtlebot_vis/start.sh: grenzen voor WSL" \
            "[wsl2]" \
            "memory=$MEM" \
            "swap=2GB" \
            "" \
            "[experimental]" \
            "# ongebruikt geheugen (cache) teruggeven aan Windows" \
            "autoMemoryReclaim=gradual" \
            "# virtuele schijf krimpt weer na het opruimen van images" \
            "sparseVhd=true" > "$CFG"
        echo "Aangemaakt. Het werkt pas na een herstart van WSL:"
        echo "    1. sluit dit venster (en VS Code)"
        echo "    2. PowerShell:  wsl --shutdown"
        echo "    3. Ubuntu opnieuw openen en ./start.sh"
        ;;
    *)
        echo "Niet aangemaakt. Later: ./wsl_check.sh, of zie de GitBook (Setup laptop)."
        ;;
esac
exit 0
