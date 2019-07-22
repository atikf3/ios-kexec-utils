.PHONY: all clean

SIGN		= ldid
TARGET		= kloader
FLAGS		= -framework IOKit -framework CoreFoundation -Wall
IGCC		?= xcrun -sdk iphoneos gcc 
ARCH		?= -arch arm64 -arch armv7 -arch armv7s
AARCH		= $(shell arch)
UNAME		= $(shell uname -s)

all: $(TARGET)

$(TARGET): *.c
		@echo "[INFO]: Compiling $(TARGET)..."
		$(IGCC) $(ARCH) -o $@ $(FLAGS) $^
		$(SIGN) -Stfp0.plist $@
		@echo "OK: Compiled $(TARGET) on $(UNAME) $(AARCH)"

clean:
	rm -f $(TARGET)
