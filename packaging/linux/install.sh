#!/bin/sh
# Install the unpacked application for the current user and add it to the applications menu.
#     tar xzf fairino-gcode-<version>-linux.tar.gz && sh fairino-gcode/install.sh
# Removing it again:  rm -r ~/.local/opt/fairino-gcode ~/.local/share/applications/fairino-gcode.desktop
set -e
here=$(cd "$(dirname "$0")" && pwd)
target="$HOME/.local/opt/fairino-gcode"
mkdir -p "$HOME/.local/opt" "$HOME/.local/share/applications" "$HOME/.local/bin"
rm -rf "$target"
cp -r "$here" "$target"
cat > "$HOME/.local/share/applications/fairino-gcode.desktop" <<DESKTOP
[Desktop Entry]
Type=Application
Name=FAIRINO G-code Runner
Comment=Preview a G-code toolpath, place it on a FAIRINO cobot and run it
Exec=$target/fairino-gcode %f
Icon=$target/_internal/fairino_gcode/resources/icon.png
Terminal=false
Categories=Engineering;Science;
MimeType=text/x-gcode;
DESKTOP
ln -sf "$target/fairino-gcode" "$HOME/.local/bin/fairino-gcode"
echo "Installed to $target"
echo "Start it from the applications menu, or with: fairino-gcode"
