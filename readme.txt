
# program
/opt/stlink/bin/st-flash write build/ch.bin 0x08000000

# debug
arm-none-eabi-gdb build/ch.elf

# in gdb
tar extended :4242
load 
continue

# console
/opt/ssterm-1.5/ssterm.py -b 9600 -d 8 -p none -t 1 --tx-nl cr /dev/ttyACM0


