#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <time.h>
#include <string.h>

#include <windows.h>

#define PROC_TIME (float)( clock() ) / (float)( CLOCKS_PER_SEC )

// comment line to test git commit

// processor architecture definition; syntactic sugar for numbers
enum stages     { fetch, fetch_aux, execute };
enum ppu_stages { standby, preload, evaluate, advance };

enum gp_regs { a, b, c, d, e, f, x, y };
enum index_regs { ix, iy };
enum address_modes { ri, rc, ai, ac };
enum arithmetic_modes { ff, lc, fc };

enum jump_conditions { never, always, carry, negative, nocarry, positive, xzero, yzero };

enum valid_opcodes {
	nop, jmp, jcr, jng, jnc, jps, jxz, jyz,
	add, sub, lsh, rsh, and, or,  not, xor,
	lda, ldb, ldc, ldd, lde, ldf, ldx, ldy,
	sta, stb, stc, std, ste, stf, stx, sty
};

enum ppu_regs {
	S0_coords, S1_coords, S2_coords, S3_coords, S4_coords, S5_coords, S6_coords, S7_coords,
	S0_index,  S1_index,  S2_index,  S3_index,  S4_index,  S5_index,  S6_index,  S7_index,
	L3_color, L0_pan, L2_pan, render_start, render_end
};

// data structures for CPU and PPU emulation
struct Decoder {
	enum stages microstep;
	uint32_t cycle_count;

	uint8_t  pc_page;
	uint16_t pc_offset;

	uint16_t instruction_word;
	uint16_t auxillary_word;

	uint8_t inst_opcode;
	uint8_t inst_type;
	bool    inst_single_word;
	uint8_t inst_verb;
	uint8_t inst_mode;

	enum index_regs inst_index_reg;
	uint8_t inst_page;

	uint8_t reg_input_a;
	uint8_t reg_input_b;
	uint8_t reg_output;

	uint16_t inst_offset;
	uint16_t inst_aux_data;
} cpu;

struct Registers {
	uint16_t file[8];
	enum gp_regs regs;
} regs;

struct ALU {
	uint16_t input_a;
	uint16_t input_b;
	uint32_t output;

	bool carry_flag;
	bool negative_flag;
} alu;

struct PPU {
	bool is_busy;
	enum ppu_stages compositor_step;

	uint8_t layer_0_index;
	uint8_t layer_1_index;
	uint8_t layer_2_index;
	uint8_t layer_3_index;

	uint8_t layer_0_offset;
	uint8_t layer_1_offset;
	uint8_t layer_2_offset;
	uint8_t layer_3_offset_u;
	uint8_t layer_3_offset_v;

	uint16_t layer_0_color;
	uint16_t layer_1_color;
	uint16_t layer_2_color;
	uint16_t layer_3_color;

	uint8_t  active_layer;
	uint8_t  compositor_u;
	uint8_t  compositor_v;
	uint16_t buffer_address;

	uint16_t layer_0_u;
	uint16_t layer_0_v;
	uint16_t layer_2_u;
	uint16_t layer_2_v;

	uint16_t compositor_tilesheet_address;
	uint16_t compositor_charsheet_address;
	uint16_t compositor_output_color;
} ppu;

// allocate memory space for devices on the system bus
// base system hardware
uint16_t firmware_rom[0x10000];  // 64K firmware (readonly at runtime)
uint16_t base_ram[0x100000];     // 1M base ram
// picture processor unit vram
uint16_t ppu_mainbuffer[0x10000]; // 64K 256x256 screen buffer (inaccessible via bus, read by frontend)
uint16_t ppu_tilesheet[0x10000];  // 64K PPU tilesheet
uint8_t  ppu_tilemap_0[0x400];    // 1K layer 0 tilemap (32x32)
uint8_t  ppu_tilemap_1[0x400];    // 1K layer 2 tilemap (32x32)
uint8_t  ppu_charmap[0x400];      // 1K layer 3 fixed charmap (32x32)
uint8_t  ppu_charsheet_0[0x400];  // 1K default charsheet (chars 0x00 >> 0x7F)
uint8_t  ppu_charsheet_1[0x400];  // 1K custom charsheet (chars 0x80 >> 0xFF)
uint16_t ppu_registers[0x14];     // 20 parameter registers
// programmable io interface registers
uint16_t pio_ports[32];           // 32 bytes reserved for MMIO
uint16_t pio_ddr[2];              // 2 bytes reserved for MMIO

// variables for address math
uint8_t calculated_address_page;
uint16_t calculated_address_offset;

// benchmarking statistics
float timer_old = 0;
float timer_new = 0;
uint64_t proc_cycle_count = 0;
float proc_elapsed_time = 0;
float framerate;

// address decoding logic for reading data from the correct device
uint16_t read_from_bus( uint8_t page, uint16_t offset ) {
	uint32_t fused_address = (page << 0x10) + offset;
	uint16_t data;

	// map for reading data
	if( page == 0x00 ) { data = firmware_rom[ offset ]; }

	if( page == 0x01 ) {
		if( offset == 0x00FF ) { data = cpu.cycle_count; }
	}

	if( page == 0x02 ) { data = ppu_tilesheet[ offset ]; }

	if( page == 0x03 ) {
		if( (offset >= 0x0000) && (offset <= 0x03FF) ) { data = ppu_tilemap_0[ offset & 0x3FF ]; }
		if( (offset >= 0x0400) && (offset <= 0x07FF) ) { data = ppu_tilemap_1[ offset & 0x3FF ]; }
		if( (offset >= 0x0800) && (offset <= 0x0BFF) ) { data = ppu_charmap[ offset & 0x3FF ]; }
		if( (offset >= 0x0C00) && (offset <= 0x0FFF) ) { data = ppu_charsheet_1[ offset & 0x3FF ]; }
		if( (offset >= 0x1000) && (offset <= 0x1014) ) { data = ppu_registers[ offset & 0x1F ]; }
	}

	if( (page >= 0x10) && (page <= 0x1F) ) { data = base_ram[ fused_address & 0x0FFFFF ]; }

	return data;
}

// address decoding logic for writing data to the correct device
void write_to_bus( uint8_t page, uint16_t offset, uint16_t data ) {
	uint32_t fused_address = (page << 0x10) + offset;
	// map for writing data
	if( page == 0x01 ) {

	}

	if( page == 0x02 ) { ppu_tilesheet[ offset ] = data; }

	if( page == 0x03 ) {
		if( (offset >= 0x0000) && (offset <= 0x03FF) ) { ppu_tilemap_0[ offset & 0x3FF ] = data; }
		if( (offset >= 0x0400) && (offset <= 0x07FF) ) { ppu_tilemap_1[ offset & 0x3FF ] = data; }
		if( (offset >= 0x0800) && (offset <= 0x0BFF) ) { ppu_charmap[ offset & 0x3FF ] = data; }
		if( (offset >= 0x0C00) && (offset <= 0x0FFF) ) { ppu_charsheet_1[ offset & 0x3FF ] = data; }
		if( (offset >= 0x1000) && (offset <= 0x1014) ) { ppu_registers[ offset & 0x1F ] = data; }
		if( offset == 0x1013 ) { ppu.is_busy = true; } // PPU start-frame signal
	}

	if( (page >= 0x10) && (page <= 0x1F) ) { base_ram[ fused_address & 0x0FFFFF ] = data; }
}

// interactive monitor for loading programs into rom before execution
void rom_preloader() {
	uint16_t hex_input = 0;
	unsigned int position = 0;
	unsigned int size = 0;
	
	// inquire for program size
	printf( "program size (words): " );
	scanf( "%u", &size );
	
	// show prompt if a program is to be loaded
	if( size > 0 ) { printf( "type or paste program below;\n"); }
	
	// retrieve program and write to rom, word by word
	for( position = 0; position < size; position++ ) {
		printf( "%06X: ", position );
		scanf( "%hx", &hex_input );
		firmware_rom[ position ] = hex_input;
	}
}

void print_disassembly() {
		switch( cpu.inst_opcode ) {
			case nop: printf( "nop " ); break; case jmp: printf( "jmp " ); break;
			case jcr: printf( "jcr " ); break; case jng: printf( "jng " ); break;
			case jnc: printf( "jnc " ); break; case jps: printf( "jps " ); break;
			case jxz: printf( "jxz " ); break; case jyz: printf( "jyz " ); break;
			case add: printf( "add " ); break; case sub: printf( "sub " ); break;
			case lsh: printf( "lsh " ); break; case rsh: printf( "rsh " ); break;
			case and: printf( "and " ); break; case or:  printf( "or  " ); break;
			case not: printf( "not " ); break; case xor: printf( "xor " ); break;
			case lda: printf( "lda " ); break; case ldb: printf( "ldb " ); break;
			case ldc: printf( "ldc " ); break; case ldd: printf( "ldd " ); break;
			case lde: printf( "lde " ); break; case ldf: printf( "ldf " ); break;
			case ldx: printf( "ldx " ); break; case ldy: printf( "ldy " ); break;
			case sta: printf( "sta " ); break; case stb: printf( "stb " ); break;
			case stc: printf( "stc " ); break; case std: printf( "std " ); break;
			case ste: printf( "ste " ); break; case stf: printf( "stf " ); break;
			case stx: printf( "stx " ); break; case sty: printf( "sty " ); break;
		};

		switch( cpu.inst_type ) {
			case 0:
			case 2:
			case 3:
				if( cpu.inst_verb != 0 ) {
				switch( cpu.inst_mode ) {
					case ri:
						printf( "ri " );
						printf( "%c, ", cpu.inst_index_reg?0x59:0x58 );
						printf( "#%02X", cpu.inst_page );
						break;
					case rc:
						printf( "rc " );
						printf( "#%02X", cpu.inst_page );
						if( !cpu.inst_single_word ) { printf( ", $%04X", cpu.inst_offset ); }
						break;
					case ai:
						printf( "ai " );
						printf( "%c, ", cpu.inst_index_reg?0x59:0x58 );
						printf( "#%02X", cpu.inst_page );
						break;
					case ac:
						printf( "ac " );
						printf( "#%02X", cpu.inst_page );
						if( !cpu.inst_single_word ) { printf( ", $%04X", cpu.inst_offset ); }
						break;
				}
				}
			break;

			case 1:;
				bool output_idx = cpu.reg_output >= 6;
				bool input1_idx = cpu.reg_input_a >= 6;
				bool input2_idx = cpu.reg_input_b >= 6;
				switch( cpu.inst_mode ) {
					case ff: printf( "ff %c <- %c, %c", cpu.reg_output +0x41 +(output_idx *0x11), cpu.reg_input_a +0x41 +(input1_idx *0x11), cpu.reg_input_b +0x41 +(input2_idx *0x11) ); break;
					case lc: printf( "lc %c <- $%04X", cpu.reg_output +0x41 +(output_idx *0x11), cpu.inst_aux_data ); break;
					case fc: printf( "fc %c <- %c, $%04X", cpu.reg_output +0x41 +(output_idx *0x11), cpu.reg_input_a +0x41 +(input1_idx *0x11), cpu.inst_aux_data ); break;
				}
				break;
		}
}

// display the CPU's status on the terminal, on an interval (requires ANSI esc. sequence recognition)
void update_terminal() {
	uint8_t  display_page;
	uint16_t display_offset;
	
	timer_new = (float)( clock() ) / (float)( CLOCKS_PER_SEC );
	if( (timer_new - timer_old) > framerate ) {
		timer_old = timer_new;

		// locate the cursor to the top-left corner
		printf( "\e[2H" );
		
		switch( cpu.microstep ) {
			case fetch:     printf( "fetch      " ); break;
			case fetch_aux: printf( "fetch_aux  " ); break;
			case execute:   printf( "execute    " ); break;
		}
		printf( "clock: %.4f MHz   ", proc_cycle_count / proc_elapsed_time / 1000000 );
		printf( "\n" );

		printf( "\e[4H" );
		printf( " program counter     instruction cache\n\e(0");
		printf( "l PC qqqqqqqqk      l INST qqkl AUX qqqk\e[E" );
		printf( "x   %02X%04X   x      x  %04X  xx  %04X  x\e[E", cpu.pc_page, cpu.pc_offset, cpu.instruction_word, cpu.auxillary_word );
		printf( "mqqqqqqqqqqqqj      mqqqqqqqqjmqqqqqqqqj\e[E" );
		printf( "\e(B" );

		printf( "\e[9H" );
		printf( " disassembly                flags\n\e(0");
		printf( "l ASM qqqqqqqqqqqqqqqqqqqqkl STAT qqqqqk\e[E" );
		printf( "x                         xx           x\e[E" );
		printf( "mqqqqqqqqqqqqqqqqqqqqqqqqqjmqqCqNqXqYqqj\e[E" );
		printf( "\e(B" );
		
		printf( "\e[11;4H" );

		print_disassembly();

		printf( "\e[11;31H" );
		printf( alu.carry_flag?     "# ":"  " ); 
		printf( alu.negative_flag?  "# ":"  " ); 
		printf( (regs.file[x] == 0)?"# ":"  " ); 
		printf( (regs.file[y] == 0)?"# ":"  " ); 

		printf( "\e[14H" );
		printf( " cpu registers\n\e(0" );
		printf( "l A qqqqqkl B qqqqqkl C qqqqqkl D qqqqqk\e[E" );
		printf( "x  %4X  xx  %4X  xx  %4X  xx  %4X  x\e[E", regs.file[a], regs.file[b], regs.file[c], regs.file[d] );
		printf( "mqqqqqqqqjmqqqqqqqqjmqqqqqqqqjmqqqqqqqqj\e[E" );
		printf( "l E qqqqqkl F qqqqqkl X qqqqqkl Y qqqqqk\e[E" );
		printf( "x  %4X  xx  %4X  xx  %4X  xx  %4X  x\e[E", regs.file[e], regs.file[f], regs.file[x], regs.file[y] );
		printf( "mqqqqqqqqjmqqqqqqqqjmqqqqqqqqjmqqqqqqqqj\e[E" );
		printf( "\e(B\n" );

		printf( "\e[2;44H memory inspect\n\e(0" );
		printf( "\e[3;44Hl BUS qqqqqqqqqqqqqqk" );
		for(int i = -8; i < 8; i++ ) {
			if( i == 0 ) { printf( "\e[7m" ); } else { printf( "\e[0m" ); }
			display_page   = cpu.pc_page + ((cpu.pc_offset + i) >> 16 );
			display_offset = cpu.pc_offset + i;
			printf( "\e[%d;44Hx   %02X%04X : %04X   x" , i +12, display_page, display_offset, read_from_bus( display_page, display_offset ));
		}
		printf( "\e[20;44Hmqqqqqqqqqqqqqqqqqqqj" );
		printf( "\e(B\n\n" );
	}
}

// legacy version of update_terminal() using no ANSI esc. sequences
void update_terminal_old() {
	uint8_t  display_page;
	uint16_t display_offset;
	
	timer_new = (float)( clock() ) / (float)( CLOCKS_PER_SEC );
	if( (timer_new - timer_old) > framerate ) {
		timer_old = timer_new;
		
		system( "cls" );
		
		printf( "microstep: ");
		switch( cpu.microstep ) {
			case fetch:     printf( "fetch    " ); break;
			case fetch_aux: printf( "fetch_aux" ); break;
			case execute:   printf( "execute  " ); break;
		}
		printf( "\tclock: %.4f MHz\n\n", proc_cycle_count / proc_elapsed_time / 1000000 );
		
		printf( "at PC = %02X%04X:\n", cpu.pc_page, cpu.pc_offset );
		
		printf( "  %04X", cpu.instruction_word );
		printf( cpu.inst_single_word? "      | " : " %04X | ", cpu.auxillary_word );
		
		print_disassembly();
		printf( "\n\n" );
		
		printf("registers: \n");
		printf("A=%04X\tB=%04X\tC=%04X\tD=%04X\n", regs.file[a], regs.file[b], regs.file[c], regs.file[d]);
		printf("E=%04X\tF=%04X\tX=%04X\tY=%04X\n", regs.file[e], regs.file[f], regs.file[x], regs.file[y]);
		printf( "\n" );
		
		printf( "flags: " );
		printf( alu.carry_flag?      "C " : "_ " );
		printf( alu.negative_flag?   "N " : "_ " );
		printf( (regs.file[x] == 0)? "X " : "_ " );
		printf( (regs.file[y] == 0)? "Y " : "_ " );
		printf( "\n\n" );
		
		printf( "memory inspect:\n" );
		for(int i = -6; i < 6; i++ ) {
			printf( (i == 0)? "> " : "  " );
			display_page   = cpu.pc_page + ((cpu.pc_offset + i) >> 16 );
			display_offset = cpu.pc_offset + i;
			printf( "%02X%04X : %04X\n", display_page, display_offset, read_from_bus( display_page, display_offset ));
		}
	}
}

// CPU state machine
void cpu_tick() {
	switch ( cpu.microstep ) {
		case fetch:
			// clear the auxillary word for visual purposes
			cpu.auxillary_word = 0x0000;

			// fetch instruction word from bus
			cpu.instruction_word = read_from_bus( cpu.pc_page, cpu.pc_offset );

			// decode the instruction into type, verb and mode
			cpu.inst_opcode = ( cpu.instruction_word & 0xF800 ) >> 11;
			cpu.inst_type = ( cpu.instruction_word & 0xC000 ) >> 14;
			cpu.inst_verb = ( cpu.instruction_word & 0x3800 ) >> 11;
			cpu.inst_mode = ( cpu.instruction_word & 0x0600 ) >> 9;
			
			// decode page number and index register from instruction
			cpu.inst_index_reg = ( cpu.instruction_word & 0x0100 ) > 8;
			cpu.inst_page = ( cpu.instruction_word & 0x00ff );
			
			// decode operands from instruction
			cpu.reg_output  = ( cpu.instruction_word & 0x01c0 ) >> 6;
			cpu.reg_input_a = ( cpu.instruction_word & 0x0038 ) >> 3;
			cpu.reg_input_b = ( cpu.instruction_word & 0x0007 );
			
			// decide next microstep based on received instruction
			if( (cpu.inst_mode == 1) || (cpu.inst_mode == 3) ) {
				cpu.inst_single_word = false;
				cpu.microstep = fetch_aux; // double-word instruction
			} else {
				cpu.inst_single_word = true;
				cpu.microstep = execute; // single-word instruction
			}
			if( (cpu.inst_type == 1) && (cpu.inst_mode == 2) ) {
				cpu.inst_single_word = false;
				cpu.microstep = fetch_aux; // double-word instruction edge case
			}
			
			// increment offset, if overflow then increment page
			if( cpu.pc_offset == 0xFFFF ) { cpu.pc_page++; }
			cpu.pc_offset++;
			break;

		case fetch_aux:
			// fetch auxillary word from bus
			cpu.auxillary_word = read_from_bus( cpu.pc_page, cpu.pc_offset );
			
			// (redundant) decode auxillary word
			cpu.inst_offset = cpu.auxillary_word;
			cpu.inst_aux_data = cpu.auxillary_word;
			cpu.microstep = execute; // advance to execution step
			
			// increment offset, if overflow then increment page
			if( cpu.pc_offset == 0xFFFF ) { cpu.pc_page++; }
			cpu.pc_offset++;
			break;

		case execute:;
			// perform address math based on instruction mode
			switch( cpu.inst_mode ) {
				case ri: // relative-index mode
					calculated_address_page   = cpu.pc_page + (( cpu.pc_offset + (int16_t)regs.file[ cpu.inst_index_reg +6 ] ) >> 16 );
					calculated_address_offset = cpu.pc_offset + (int16_t)regs.file[ cpu.inst_index_reg +6 ];
					break;
				case rc: // relative-constant mode
					calculated_address_page   = cpu.pc_page + (( cpu.pc_offset + (int16_t)cpu.inst_offset ) >> 16 );
					calculated_address_offset = cpu.pc_offset + (int16_t)cpu.inst_offset;
					break;
				case ai: // absolute-index mode
					calculated_address_page   = cpu.inst_page;
					calculated_address_offset = regs.file[ cpu.inst_index_reg +6 ];
					break;
				case ac: // absolute-constant mode
					calculated_address_page   = cpu.inst_page;
					calculated_address_offset = cpu.inst_offset;
					break;
			}
			
			// sort operations by type
			switch( cpu.inst_type ) {
				case 0:; // PC manipulation aka. branching
					bool condition_is_true = 0;
					switch( cpu.inst_verb ) {
						case never :   break; // does nothing
						case always:   condition_is_true = 1; // simple goto
						case carry:    if ( alu.carry_flag )     { condition_is_true = 1; } break; // if carry flag is set then jump
						case negative: if ( alu.negative_flag )  { condition_is_true = 1; } break; // if negative flag is set then jump
						case nocarry:  if ( !alu.carry_flag )    { condition_is_true = 1; } break; // if carry flag is not set then jump
						case positive: if ( !alu.negative_flag ) { condition_is_true = 1; } break; // if negative flag is not set then jump
						case xzero:    if ( regs.file[x] == 0 )  { condition_is_true = 1; } break; // if register X == 0 then jump
						case yzero:    if ( regs.file[y] == 0 )  { condition_is_true = 1; } break; // if register Y == 0 then jump
					}
					if( condition_is_true ) { cpu.pc_page = calculated_address_page; cpu.pc_offset = calculated_address_offset; }
				break;
				case 1: // load constants to registers or perform arithmetic operations
					alu.input_a = regs.file[cpu.reg_input_a];
					alu.input_b = regs.file[cpu.reg_input_b];
					switch( cpu.inst_mode ) {
						case ff: // register-register arithmetic
							switch( cpu.inst_verb +8 ) {
								case add: alu.output =  alu.input_a +  alu.input_b; break;
								case sub: alu.output =  alu.input_a -  alu.input_b; break;
								case lsh: alu.output =  alu.input_a << alu.input_b; break;
								case rsh: alu.output =  alu.input_a >> alu.input_b; break;
								case and: alu.output =  alu.input_a &  alu.input_b; break;
								case or:  alu.output =  alu.input_a |  alu.input_b; break;
								case not: alu.output = ~alu.input_a;                break;
								case xor: alu.output =  alu.input_a ^  alu.input_b; break;
							}
						break;
						case lc: alu.output = cpu.inst_aux_data; // load constant
						case fc: // register-register arithmetic
							switch( cpu.inst_verb +8 ) {
								case add: alu.output =  alu.input_a +  cpu.inst_aux_data; break;
								case sub: alu.output =  alu.input_a -  cpu.inst_aux_data; break;
								case lsh: alu.output =  alu.input_a << cpu.inst_aux_data; break;
								case rsh: alu.output =  alu.input_a >> cpu.inst_aux_data; break;
								case and: alu.output =  alu.input_a &  cpu.inst_aux_data; break;
								case or:  alu.output =  alu.input_a |  cpu.inst_aux_data; break;
								case not: alu.output = ~alu.input_a;                      break;
								case xor: alu.output =  alu.input_a ^  cpu.inst_aux_data; break;
							}
						break;
					}
					regs.file[cpu.reg_output] = alu.output;
					switch( cpu.inst_verb +8 ) {
						case add: alu.carry_flag    = ( alu.output & 0xFFFF0000 >> 16 ) > 0; break;
						case sub: alu.negative_flag = ( alu.output & 0xFFFF0000 >> 16 ) > 0; break;
					}
				break;
				case 2: // load data from memory to register
					regs.file[cpu.inst_verb] = read_from_bus( calculated_address_page, calculated_address_offset );
				break;
				case 3: // store data from register to memory
					write_to_bus( calculated_address_page, calculated_address_offset, regs.file[cpu.inst_verb] );
				break;
			}
			
			// continue fetching next instruction word
			cpu.microstep = fetch;
		break;
	}
}

// PPU state machine
void ppu_tick() {
	switch( ppu.compositor_step ) {
		case standby:
			// wait for start signal to start processing frame data
			if( ppu.is_busy == true ) { ppu.compositor_step = preload; }
			break;
		case preload:
			// update uv coordinates of all layers at the current pixel
			ppu.layer_0_u = ( ppu.compositor_u + (uint8_t)(ppu_registers[L0_pan])      ) & 0x01ff;
			ppu.layer_0_v = ( ppu.compositor_v + (uint8_t)(ppu_registers[L0_pan] >> 8) ) & 0x01ff;
			ppu.layer_2_u = ( ppu.compositor_u + (uint8_t)(ppu_registers[L2_pan])      ) & 0x01ff;
			ppu.layer_2_v = ( ppu.compositor_v + (uint8_t)(ppu_registers[L2_pan] >> 8) ) & 0x01ff;
			
			// update tile indexes and offsets of all layers
			ppu.layer_0_index  = (ppu.layer_0_u >> 4) | ( (ppu.layer_0_v & 0x1f0) << 1);
			ppu.layer_0_offset = (ppu.layer_0_u & 0x0f) | ( ( ppu.layer_0_v & 0x0f ) << 4 );
			
			// checking the boundaries of all sprites
			bool S0_active = (ppu.compositor_u >= (ppu_registers[S0_coords] & 0xFF) ) && (ppu.compositor_u < ( (ppu_registers[S0_coords] & 0xFF) + 0x10 ) ) && (ppu.compositor_v >= (ppu_registers[S0_coords] >> 8) ) && (ppu.compositor_v < ( (ppu_registers[S0_coords] >> 15) + 0x10 ) );
			bool S1_active = (ppu.compositor_u >= (ppu_registers[S1_coords] & 0xFF) ) && (ppu.compositor_u < ( (ppu_registers[S1_coords] & 0xFF) + 0x10 ) ) && (ppu.compositor_v >= (ppu_registers[S1_coords] >> 8) ) && (ppu.compositor_v < ( (ppu_registers[S1_coords] >> 15) + 0x10 ) );
			bool S2_active = (ppu.compositor_u >= (ppu_registers[S2_coords] & 0xFF) ) && (ppu.compositor_u < ( (ppu_registers[S2_coords] & 0xFF) + 0x10 ) ) && (ppu.compositor_v >= (ppu_registers[S2_coords] >> 8) ) && (ppu.compositor_v < ( (ppu_registers[S2_coords] >> 15) + 0x10 ) );
			bool S3_active = (ppu.compositor_u >= (ppu_registers[S3_coords] & 0xFF) ) && (ppu.compositor_u < ( (ppu_registers[S3_coords] & 0xFF) + 0x10 ) ) && (ppu.compositor_v >= (ppu_registers[S3_coords] >> 8) ) && (ppu.compositor_v < ( (ppu_registers[S3_coords] >> 15) + 0x10 ) );
			bool S4_active = (ppu.compositor_u >= (ppu_registers[S4_coords] & 0xFF) ) && (ppu.compositor_u < ( (ppu_registers[S4_coords] & 0xFF) + 0x10 ) ) && (ppu.compositor_v >= (ppu_registers[S4_coords] >> 8) ) && (ppu.compositor_v < ( (ppu_registers[S4_coords] >> 15) + 0x10 ) );
			bool S5_active = (ppu.compositor_u >= (ppu_registers[S5_coords] & 0xFF) ) && (ppu.compositor_u < ( (ppu_registers[S5_coords] & 0xFF) + 0x10 ) ) && (ppu.compositor_v >= (ppu_registers[S5_coords] >> 8) ) && (ppu.compositor_v < ( (ppu_registers[S5_coords] >> 15) + 0x10 ) );
			bool S6_active = (ppu.compositor_u >= (ppu_registers[S6_coords] & 0xFF) ) && (ppu.compositor_u < ( (ppu_registers[S6_coords] & 0xFF) + 0x10 ) ) && (ppu.compositor_v >= (ppu_registers[S6_coords] >> 8) ) && (ppu.compositor_v < ( (ppu_registers[S6_coords] >> 15) + 0x10 ) );
			bool S7_active = (ppu.compositor_u >= (ppu_registers[S7_coords] & 0xFF) ) && (ppu.compositor_u < ( (ppu_registers[S7_coords] & 0xFF) + 0x10 ) ) && (ppu.compositor_v >= (ppu_registers[S7_coords] >> 8) ) && (ppu.compositor_v < ( (ppu_registers[S7_coords] >> 15) + 0x10 ) );
			
			// S0 = top of sprite stack
			if( S0_active )      { ppu.layer_1_index = ppu_registers[S0_index]; }
			else if( S1_active ) { ppu.layer_1_index = ppu_registers[S1_index]; }
			else if( S2_active ) { ppu.layer_1_index = ppu_registers[S2_index]; }
			else if( S3_active ) { ppu.layer_1_index = ppu_registers[S3_index]; }
			else if( S4_active ) { ppu.layer_1_index = ppu_registers[S4_index]; }
			else if( S5_active ) { ppu.layer_1_index = ppu_registers[S5_index]; }
			else if( S6_active ) { ppu.layer_1_index = ppu_registers[S6_index]; }
			else if( S7_active ) { ppu.layer_1_index = ppu_registers[S7_index]; }
			else { ppu.layer_1_index = 0x00; } // fallback if no sprites are active
			ppu.layer_2_index  = (ppu.layer_2_u >> 4) | ( (ppu.layer_2_v & 0x1f0) << 1);
			ppu.layer_2_offset = (ppu.layer_0_u & 0x0f) | ( ( ppu.layer_0_v & 0x0f ) << 4 );
			ppu.layer_3_index  = (ppu.compositor_u >> 3) | ( (ppu.compositor_v & 0xf8) << 2);
			ppu.layer_3_offset_u = ppu.compositor_u & 0x07;
			ppu.layer_3_offset_v = ppu.compositor_v & 0x07;
			
			// advance to next step
			ppu.compositor_step = evaluate;
			break;
		case evaluate:
			// prioritizes top layer with non-zero tile index
			if( ppu.layer_3_index != 0 )       { ppu.active_layer = 3; }
			else if ( ppu.layer_2_index != 0 ) { ppu.active_layer = 2; }
			else if ( ppu.layer_1_index != 0 ) { ppu.active_layer = 1; }
			else if ( ppu.layer_0_index != 0 ) { ppu.active_layer = 0; }
			else { ppu.active_layer = 0; } // fallback if no layers are active

			// retrieve pixel color data at current coordinates
			// layer 3 gets special treatment because it is a different tilesheet format
			if( ppu.active_layer == 3 ) {
				// compute address of pixel being requested
				ppu.compositor_charsheet_address = ( (ppu.layer_3_index & 0xf0) << 7) | (ppu.layer_3_offset_v << 4) | (ppu.layer_3_index & 0x0f);
				
				// read from correct charmap according to the MSB of L3_index and L3_color
				if( ppu.layer_3_index >> 7 == 1 ) {
					if( ppu_registers[L3_color] >> 15 == 1) {
						// use user-defined charmap
						ppu.compositor_output_color = ppu_charsheet_1[ppu.compositor_charsheet_address];
					} else {
						// use inverted version of default charmap
						ppu.compositor_output_color = ~ppu_charsheet_0[ppu.compositor_charsheet_address];
					}
				} else {
					// use default charmap
					ppu.compositor_output_color = ppu_charsheet_0[ppu.compositor_charsheet_address];
				}
			} else {
				// compute address of pixel being requested
				switch( ppu.active_layer ) {
					case 0: ppu.compositor_tilesheet_address = (ppu.layer_0_index << 8) | ppu.layer_0_offset; break;
					case 1: ppu.compositor_tilesheet_address = (ppu.layer_1_index << 8) | ppu.layer_1_offset; break;
					case 2: ppu.compositor_tilesheet_address = (ppu.layer_2_index << 8) | ppu.layer_2_offset; break;
				}
				
				// read pixel color at designated address
				ppu.compositor_output_color = ppu_tilesheet[ppu.compositor_tilesheet_address];
			}
			
			if( (ppu.active_layer > 0 ) && (ppu.compositor_output_color == 0x0000) ) {
				// if retrieved pixel = transparent then re-evaluate one layer down, until active layer = 0
				ppu.active_layer--;
			} else {
				// otherwise, output color to screen buffer
				ppu.compositor_step = advance;
			}
			break;
		case advance:
			// calculate pixel address and write processed pixel to framebuffer
			ppu.buffer_address = (ppu.compositor_v << 8) | ppu.compositor_u;
			ppu_mainbuffer[ppu.buffer_address] = ppu.compositor_output_color;
			
			// advance compositor coordinates
			if( ppu.compositor_u = 0xFF ) { ppu.compositor_v++; }
			ppu.compositor_u++;

			// enter standby mode if compositor_uv +1 = render_end
			if( (ppu.buffer_address +1) == ppu_registers[render_end] ) {
				ppu.compositor_step = standby;
				ppu.is_busy = false;
			} else {
				ppu.compositor_step = preload;
			}
			break;
	}
}

int main(int argc, char* argv[]) {
	// reset all registers in system
	cpu.microstep = fetch;
	cpu.pc_page = 0x00;
	cpu.pc_offset = 0x0000;
	cpu.instruction_word = 0x0000;
	cpu.auxillary_word = 0x0000;
	cpu.inst_single_word = false;
	cpu.inst_type = 0x00;
	cpu.inst_verb = 0x00;
	cpu.inst_mode = 0x00;
	cpu.inst_index_reg = x;
	cpu.inst_page = 0x00;
	cpu.reg_input_a = 0x00;
	cpu.reg_input_b = 0x00;
	cpu.reg_output = 0x00;
	cpu.inst_offset = 0x0000;
	cpu.inst_aux_data = 0x0000;

	regs.file[a] = 0x0000;
	regs.file[b] = 0x0000;
	regs.file[c] = 0x0000;
	regs.file[d] = 0x0000;
	regs.file[e] = 0x0000;
	regs.file[f] = 0x0000;
	regs.file[x] = 0x0000;
	regs.file[y] = 0x0000;

	alu.input_a = 0x0000;
	alu.input_b = 0x0000;
	alu.output = 0x0000;
	alu.carry_flag = false;
	alu.negative_flag = false;

	ppu.is_busy = false;
	ppu.compositor_step = standby;
	ppu.layer_0_index = 0x00;
	ppu.layer_0_offset = 0x00;
	ppu.layer_0_color = 0x0000;
	ppu.layer_1_index = 0x00;
	ppu.layer_1_offset = 0x00;
	ppu.layer_1_color = 0x0000;
	ppu.layer_2_index = 0x00;
	ppu.layer_2_offset = 0x00;
	ppu.layer_2_color = 0x0000;
	ppu.layer_3_index = 0x00;
	ppu.layer_3_offset_u = 0x00;
	ppu.layer_3_offset_v = 0x00;
	ppu.layer_3_color = 0x0000;
	ppu.active_layer = 0;
	ppu.compositor_u = 0x00;
	ppu.compositor_v = 0x00;
	ppu.buffer_address = 0x0000;
	ppu.layer_0_u = 0x0000;
	ppu.layer_0_v = 0x0000;
	ppu.layer_2_u = 0x0000;
	ppu.layer_2_v = 0x0000;
	ppu.compositor_tilesheet_address = 0x0000;
	ppu.compositor_charsheet_address = 0x000;
	ppu.compositor_output_color = 0x0000;
	
	// look for -no-ansi flag in program arguments
	bool enable_ansi = true;
	if( argc > 1 ) {
		// if first argument is "-no-ansi"
		if( strcmp( argv[1], "-no-ansi" ) == 0 ) {
			// disable escape sequence usage
			enable_ansi = false;
		}
	}
	
	// sets correct target FPS for different display modes
	// 30 FPS for the graphical version or 5 FPS for the text mode version
	framerate = enable_ansi? 0.0166667 : 0.2;
	
	rom_preloader(); // rom loader for user programs
	
	if( enable_ansi ) { printf( "\e[2J" ); } // clears screen
	
	bool running = true;
	do {
		// update terminal screen with debug processor data
		// runs the correct update routine for different windows versions
		if( enable_ansi ) {
			update_terminal();
		} else {
			update_terminal_old();
		}
	
		cpu_tick(); // advance CPU state machine
		ppu_tick(); // advance PPU state machine
		
		proc_cycle_count++; proc_elapsed_time = PROC_TIME; // benchmarking stats
		cpu.cycle_count = proc_cycle_count & 0xFFFFFF; // update cycle counter
		
		if( GetAsyncKeyState( VK_SPACE ) ) {
			running = false;
		}
	} while (running == true);

	return 0;
}
