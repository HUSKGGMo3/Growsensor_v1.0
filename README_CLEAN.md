# PlatformIO Cache Reset

Nutze die folgenden Schritte, um alle PlatformIO-Artefakte sauber zu entfernen und einen frischen Build zu erzwingen:

1. ```bash
   pio run --target clean
   ```
2. ```bash
   pio run --target cleanall
   ```
3. Lösche anschließend das lokale Build-Verzeichnis:
   ```bash
   rm -rf .pio/
   ```
