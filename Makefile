RTL = rtl
BUILD = build
PACKAGES = ./src/:+
SIM_FILE = ./build/mkTop_sim
TOP = src/Soc.bsv


LIB = \
			$(BLUESPECDIR)/Verilog/SizedFIFO.v \
			$(BLUESPECDIR)/Verilog/FIFO1.v \
			$(BLUESPECDIR)/Verilog/FIFO2.v \
			$(BLUESPECDIR)/Verilog/BRAM1.v \
			$(BLUESPECDIR)/Verilog/BRAM2.v \
			$(BLUESPECDIR)/Verilog/BRAM1BE.v \
			$(BLUESPECDIR)/Verilog/BRAM2BE.v \
			$(BLUESPECDIR)/Verilog/RevertReg.v \
			$(BLUESPECDIR)/Verilog/RegFile.v \
			$(BLUESPECDIR)/Verilog/RegFileLoad.v \
			src/Top.v \
			src/dvi.v \
			src/tmds_encoder.v \
			src/clk_25_system.v \
			src/fake_differential.v \
			src/vga2dvid.v
			#src/sdram_axi.v \
			#src/sd_card.v

DOT_FILES = $(shell ls ./build/*_combined_full.dot) \
	$(shell ls ./build/*_conflict.dot)

svg:
	$(foreach f, $(DOT_FILES), sed -i '/_init_register_file/d' $(f);)
	$(foreach f, $(DOT_FILES), sed -i '/_update_register_file/d' $(f);)
	$(foreach f, $(DOT_FILES), sed -i '/_ehr_canon/d' $(f);)
	$(foreach f, $(DOT_FILES), sed -i '/_block_ram_apply_read/d' $(f);)
	$(foreach f, $(DOT_FILES), sed -i '/_block_ram_apply_write/d' $(f);)
	$(foreach f, $(DOT_FILES), sed -i '/Sched /d' $(f);)
	$(foreach f, $(DOT_FILES), dot -Tsvg $(f) > $(f:.dot=.svg);)

test:
	tests/elf_to_hex/elf_to_hex ./tests/code/zig-out/bin/zig-unix.elf ./tests/Mem.hex
	riscv32-none-elf-objdump ./tests/code/zig-out/bin/zig-unix.elf -D > ./tests/code/firmware.asm

compile:
	bsc \
		-verilog \
		-vdir $(RTL) -bdir $(BUILD) -info-dir $(BUILD) \
		-no-warn-action-shadowing -check-assert \
		-keep-fires -aggressive-conditions -show-schedule -sched-dot \
		-cpp +RTS -K128M -RTS  -show-range-conflict \
		-p $(PACKAGES) -g mkCPU -u $(TOP)


link:
	bsc -e mkCPU -verilog -o $(SIM_FILE) -vdir $(RTL) -bdir $(BUILD) \
		-info-dir $(BUILD) -vsim iverilog $(RTL)/mkCPU.v

yosys:
	yosys \
		-DULX3S -q -p "synth_ecp5 -abc9 -top mkTop -json ./build/mkTop.json; prep; show -stretch -prefix count -format dot" \
		rtl/* $(LIB)

		#-DULX3S -q -p "synth_ecp5 -noabc9 -top mkTop -json ./build/mkTop.json" \
		#-DULX3S -q -p "synth_ecp5 -abc9 -top mkTop -json ./build/mkTop.json" \

yosys_ice40:
	yosys \
		-p "synth_ice40 -top mkTop -json ./build/mkTop.json" \
		src/Top.v src/sdram_axi.v rtl/* $(LIB)

nextpnr:
	nextpnr-ecp5 --force --timing-allow-fail --json ./build/mkTop.json --lpf ulx3s.lpf \
		--textcfg ./build/mkTop_out.config --85k --package CABGA381

nextpnr_ice40:
	nextpnr-ice40 --up5k --timing-allow-fail --json ./build/mkTop.json --pcf ./Pins.pcf --asc build/mkTop.asc --package sg48

nextpnr_gui:
	nextpnr-ecp5 --force --timing-allow-fail --json ./build/mkTop.json --lpf ulx3s.lpf \
		--textcfg ./build/mkTop_out.config --85k --freq 40 --package CABGA381 --gui

ecppack:
	ecppack --compress --svf-rowsize 100000 --svf ./build/mkTop.svf \
		./build/mkTop_out.config ./build/mkTop.bit

ram_simulate:
	iverilog -o build/test_sdram.vvp -s test_sdram \
		simulation/test_sdram.v rtl/* simulation/mt48lc16m16a2.v $(LIB)
	vvp build/test_sdram.vvp

simulate: link
	$(SIM_FILE)

prog:
	sudo fujprog build/mkTop.bit

prog_t:
	sudo fujprog build/mkTop.bit -t

clean:
	rm -rf $(BUILD)/*
	rm -rf $(RTL)/*
