@ECHO OFF
"C:\Program Files\Atmel\AVR Tools\AvrAssembler2\avrasm2.exe" -S "E:\Projects\Soft for AVR\Watch\Firmware\labels.tmp" -fI -W+ie -C V2E -o "E:\Projects\Soft for AVR\Watch\Firmware\Watch.hex" -d "E:\Projects\Soft for AVR\Watch\Firmware\Watch.obj" -e "E:\Projects\Soft for AVR\Watch\Firmware\Watch.eep" -m "E:\Projects\Soft for AVR\Watch\Firmware\Watch.map" "E:\Projects\Soft for AVR\Watch\Firmware\Watch.asm"
