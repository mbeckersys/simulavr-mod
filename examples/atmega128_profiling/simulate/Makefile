# hey emacs, this is a -*- makefile -*-
ELF=main.elf
MCU = atmega128

SIM2TIMES=../../../scripts/simulavr2times.py

.PHONY: simulate

all: simulate

simulate: $(ELF)
	python -u $(SIM2TIMES) -s='-l 0 -d $(MCU) -T exit' -e $(ELF) --pretty --coverage | tee alltimes

