; Inno Setup script: turns dist\fairino-gcode\ (made by packaging\build_app.py on Windows) into Setup.exe.
;     iscc /DAppVersion=0.1.0 packaging\windows\installer.iss
; NOT TESTED: written without access to a Windows machine. Check it before the first release.
#ifndef AppVersion
  #define AppVersion "0.1.0"
#endif
[Setup]
AppId={{6F1B2C0E-8B5E-4F0B-9C1B-0A6F6C0D51A7}
AppName=FAIRINO G-code Runner
AppVersion={#AppVersion}
AppPublisher=Devonics
DefaultDirName={autopf}\FAIRINO G-code Runner
DefaultGroupName=FAIRINO G-code Runner
OutputDir=..\..\dist
OutputBaseFilename=fairino-gcode-{#AppVersion}-setup
Compression=lzma2
SolidCompression=yes
ArchitecturesInstallIn64BitMode=x64compatible
LicenseFile=..\..\THIRD_PARTY_NOTICES.md

[Files]
Source: "..\..\dist\fairino-gcode\*"; DestDir: "{app}"; Flags: recursesubdirs ignoreversion

[Icons]
Name: "{group}\FAIRINO G-code Runner"; Filename: "{app}\fairino-gcode.exe"
Name: "{autodesktop}\FAIRINO G-code Runner"; Filename: "{app}\fairino-gcode.exe"; Tasks: desktopicon

[Tasks]
Name: "desktopicon"; Description: "Create a desktop shortcut"; Flags: unchecked

[Run]
Filename: "{app}\fairino-gcode.exe"; Description: "Start FAIRINO G-code Runner"; Flags: nowait postinstall skipifsilent
