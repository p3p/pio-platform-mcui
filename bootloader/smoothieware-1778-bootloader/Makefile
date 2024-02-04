#
#
#

APPBAUD  = 115200

PROJECT  = DFU-Bootloader

CONSOLE  = /dev/arduino

CSRC     = $(wildcard ./src/*.c)
CXXSRC   = $(wildcard ./src/*.cpp)
ASRC     = $(wildcard ./src/*.S)

SUBDIRS  = 

INC      = . $(shell find */ -type d)

LIBRARIES =

OUTDIR   = build/obj
BINDIR   = build

OSRC     =

NXPSRC   = $(shell find lib/CMSISv2p00_LPC17xx/src lib/LPC177x_8xLib/src -name '*.c')
NXPO     = $(patsubst %.c,$(OUTDIR)/%.o,$(notdir $(NXPSRC))) $(OUTDIR)/system_LPC177x_8x.o

FATFSSRC = $(shell find lib/fatfs/src -name '*.c')
FATFSO   = $(patsubst %.c,$(OUTDIR)/%.o,$(notdir $(FATFSSRC)))

CHIP     = lpc4078
MCU      = cortex-m4

TOOLCHAIN= ~/.platformio/packages/toolchain-gccarmnoneeabi
ARCH     = arm-none-eabi
PREFIX   = $(TOOLCHAIN)/bin/$(ARCH)-

CC       = $(PREFIX)gcc
# CXX      = $(PREFIX)g++
OBJCOPY  = $(PREFIX)objcopy
OBJDUMP  = $(PREFIX)objdump
AR       = $(PREFIX)ar
SIZE     = $(PREFIX)size
READELF  = $(PREFIX)readelf

# You MUST link with G++ if you have even one C++ source file in the project
# If you have no C++, then feel free to link with gcc which gives a significant reduction in included library code
LINK     = $(PREFIX)gcc

MKDIR    = mkdir -p
RMDIR    = rmdir
RM       = rm -f

OPTIMIZE = s

#DEBUG_MESSAGES
CDEFS    = MAX_URI_LENGTH=512 __LPC177X_8X__ USB_DEVICE_ONLY APPBAUD=$(APPBAUD)

FLAGS    = -O$(OPTIMIZE) -mcpu=$(MCU) -mthumb -mthumb-interwork -mlong-calls -ffunction-sections -fdata-sections -Wall -g -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
FLAGS   += $(patsubst %,-I%,$(INC))
FLAGS   += $(patsubst %,-D%,$(CDEFS))
CFLAGS   = $(FLAGS) -std=gnu99 -pipe -fno-builtin-printf -fno-builtin-fprintf -fno-builtin-vfprintf -fno-builtin-puts
ASFLAGS  = $(FLAGS)
CXXFLAGS = $(FLAGS) -fno-rtti -fno-exceptions -std=gnu++11

LDFLAGS  = $(FLAGS) -Wl,--as-needed,--gc-sections,-e,__cs3_reset_cortex_m,-T,./src/$(CHIP).ld
LDFLAGS += $(patsubst %,-L%,$(LIBRARIES)) -lc
LDFLAGS += -Wl,-Map=$(OUTDIR)/$(PROJECT).map

OBJ      = $(patsubst %,$(OUTDIR)/%,$(notdir $(CSRC:.c=.o) $(CXXSRC:.cpp=.o) $(ASRC:.S=.o)))

VPATH    = . $(patsubst %/inc,%/src,$(INC)) $(dir $(NXPSRC)) $(dir $(USBSRC)) $(dir $(UIPSRC)) $(dir $(LWIPSRC))

.PHONY: all clean program upload size functions functionsizes

.PRECIOUS: $(OBJ)

all: $(OUTDIR) $(OUTDIR)/nxp.ar $(OUTDIR)/fatfs.ar $(BINDIR)/$(PROJECT).elf $(BINDIR)/$(PROJECT).bin $(BINDIR)/$(PROJECT).hex size

clean:
	@echo "  RM    " ".o"
	@$(RM) $(OBJ) $(OBJ:%.o=%.lst)

	@echo "  RM    " "nxp"
	@$(RM) $(NXPO) $(NXPO:%.o=%.lst) $(OUTDIR)/nxp.ar

	@echo "  RM    " "fatfs"
	@$(RM) $(FATFSO) $(FATFSO:%.o=%.lst) $(OUTDIR)/fatfs.ar

	@echo "  RM    " "build/"$(PROJECT)".*"
	@$(RM) $(BINDIR)/$(PROJECT).bin $(BINDIR)/$(PROJECT).hex $(BINDIR)/$(PROJECT).elf $(OUTDIR)/$(PROJECT).map

	@echo "  RM    " "build/obj"
	@$(RMDIR) $(OUTDIR)

	@echo "  RM    " "build/"
	@$(RMDIR) $(BINDIR); true

program: $(BINDIR)/$(PROJECT).hex
	lpc21isp $^ $(CONSOLE) 115200 12000

upload: program

program-jlink: $(BINDIR)/$(PROJECT).bin
	@echo "h\nloadbin $^, 0x0\nr\nq" > $(OUTDIR)/jlink.script
	@JLinkExe -device $(CHIP) -speed 4000 -if swd -autoconnect 1 -CommanderScript $(OUTDIR)/jlink.script

upload-jlink: size program-jlink

console:
	@stty raw ignbrk -echo $(APPBAUD) < $(CONSOLE)
	@echo "Press ctrl+D to exit"
	@( cat <&3 & cat >&3 ; kill %% ) 3<>$(CONSOLE)


# size: $(OUTDIR)/$(PROJECT).elf
# 	@$(SIZE) $<
size: $(BINDIR)/$(PROJECT).elf
	@echo
	@echo $$'           \033[1;4m  SIZE        LPC1778         (bootloader)\033[0m'
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(text|rodata)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "  FLASH    %6d bytes  %2d%% of %3dkb    %2d%% of %3dkb\n", $$a, ceil($$a * 100 / (512 * 1024)), 512, ceil($$a * 100 / (16 * 1024)), 16 }'
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(data|bss)\s+([0-9a-f]+)/    && do { $$a += eval "0x$$2" }; END { printf "  RAM      %6d bytes  %2d%% of %3dkb\n", $$a, ceil($$a * 100 / ( 16 * 1024)),  16 }'

functions: $(BINDIR)/$(PROJECT).elf
	@$(READELF) -s $^ | perl -e 'for (<>) { /^\s+(\d+):\s*([0-9A-F]+)\s+(\d+)/i && do { s/^\s+//; push @symbols, [ split /\s+/, $$_ ]; }; }; for (sort { hex($$a->[1]) <=> hex($$b->[1]); } @symbols) { printf "0x%08s: [%4d] %7s %s\n", $$_->[1], $$_->[2], $$_->[3], $$_->[7] if ($$_->[2]) && (hex($$_->[1]) < 0x10000000); }'

functionsizes: $(BINDIR)/$(PROJECT).elf
	@$(READELF) -s $^ | perl -e 'for (<>) { /^\s+(\d+):\s*([0-9A-F]+)\s+(\d+)/i && do { s/^\s+//; push @symbols, [ split /\s+/, $$_ ]; }; }; for (sort { $$a->[2] <=> $$b->[2]; } @symbols) { printf "0x%08s: [%4d] %7s %s\n", $$_->[1], $$_->[2], $$_->[3], $$_->[7] if ($$_->[2]) && (hex($$_->[1]) < 0x10000000); }'

$(OUTDIR):
	@$(MKDIR) $(OUTDIR)

$(BINDIR):
	@$(MKDIR) $(BINDIR)

$(BINDIR)/%.bin: $(BINDIR)/%.elf
	@echo "  COPY  " $@
	@$(OBJCOPY) -O binary $< $@

$(BINDIR)/%.hex: $(BINDIR)/%.elf
	@echo "  COPY  " $@
	@$(OBJCOPY) -O ihex $< $@

$(OUTDIR)/%.sym: $(BINDIR)/%.elf
	@echo "  SYM   " $@
	@$(OBJDUMP) -t $< | perl -ne 'BEGIN { printf "%6s  %-40s %s\n", "ADDR","NAME","SIZE"; } /([0-9a-f]+)\s+(\w+)\s+O\s+\.(bss|data)\s+([0-9a-f]+)\s+(\w+)/ && printf "0x%04x  %-40s +%d\n", eval("0x$$1") & 0xFFFF, $$5, eval("0x$$4")' | sort -k1 > $@

$(BINDIR)/%.elf: $(OBJ) $(OUTDIR)/nxp.ar $(OUTDIR)/fatfs.ar
	@echo "  LINK  " $@
	@$(LINK) $(OSRC) -Wl,-Map=$(@:.elf=.map) -o $@ $^ $(LDFLAGS)

$(OUTDIR)/%.o: %.c Makefile
	@echo "  CC    " $@
	@$(CC) $(CFLAGS) -Wa,-adhlns=$(@:.o=.lst) -c -o $@ $<

# $(OUTDIR)/%.o: %.cpp
# 	@echo "  CXX   " $@
# 	@$(CXX) $(CXXFLAGS) -Wa,-adhlns=$(@:.o=.lst) -c -o $@ $<

$(OUTDIR)/%.o: %.S Makefile
	@echo "  AS    " $@
	@$(CC) $(ASFLAGS) -Wa,-adhlns=$(@:.o=.lst) -c -o $@ $<

$(OUTDIR)/nxp.ar: $(NXPO)
	@echo "  AR    " "  nxp/"$@
	@$(AR) cru $@ $^

$(OUTDIR)/fatfs.ar: $(FATFSO)
	@echo "  AR    " "fatfs/"$@
	@$(AR) cru $@ $^
