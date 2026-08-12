# Makefile -- build and run the host-side (PC) tests for the portable
# EDDI-MCU algorithm modules. This builds against gcc for x86, NOT the
# ARM target -- it's here to verify the algorithms, the same role
# fpga/sim/run_all.sh plays for the RTL. Real embedded build (CMSIS-DSP,
# HAL, linker scripts) is separate, later work.

CC      = gcc
CFLAGS  = -std=c11 -Wall -Wextra -O2 -I../src/common
LDFLAGS = -lm

COMMON  = ../src/common/debris_detector.c \
          ../src/common/framed_protocol.c \
          ../src/common/qtn_pipeline.c \
          ../src/common/fft_backend_ref.c

.PHONY: all test clean

all: test

test: test_debris test_framed test_qtn
	@echo "================ test_debris ================"
	@./test_debris
	@echo "================ test_framed ================"
	@./test_framed
	@echo "================ test_qtn ================"
	@./test_qtn
	@echo "====================================="
	@echo "ALL TESTS PASSED"

test_debris: test_debris.c $(COMMON)
	$(CC) $(CFLAGS) -o $@ test_debris.c ../src/common/debris_detector.c $(LDFLAGS)

test_framed: test_framed.c $(COMMON)
	$(CC) $(CFLAGS) -o $@ test_framed.c ../src/common/framed_protocol.c $(LDFLAGS)

test_qtn: test_qtn.c $(COMMON)
	$(CC) $(CFLAGS) -o $@ test_qtn.c ../src/common/qtn_pipeline.c ../src/common/fft_backend_ref.c $(LDFLAGS)

clean:
	rm -f test_debris test_framed test_qtn *.exe
