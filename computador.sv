//MEMORIA.V


//PROGRAM MEMORY 
module programmemory (
    input wire [7:0] address,  // Endereço da ROM
    input wire clock,          // Clock
    output reg [7:0] data_out // Dados de saída
    //input wire [7:0] data_in,  // Dados de entrada para escrita
    //input wire write_enable    // Sinal para permitir escrita na memória
);
    reg [7:0] prmemo [0:127]; // Vetor de memória para armazenar múltiplos bytes 
    integer file, status;
    reg [7:0] line;           // Buffer para armazenar cada linha lida
    integer EN;
    integer i;

    // Inicialização da memória de programa a partir de um arquivo
    initial begin
        file = $fopen("input.hex", "rb");
        if (file == 0) begin
            $display("Erro: Não foi possível abrir o arquivo.");
            $finish;
        end

       // $display("Lendo o arquivo linha por linha:");
        i = 0;
        while (!$feof(file)) begin
            status = $fscanf(file, "%b\n", prmemo[i]);  // Lê 1 byte de cada vez
            if (status != 0) begin
               // $display("Linha lida: %h", prmemo[i]);
            end
            i = i + 1;
        end
        $fclose(file);
    end

    // Lógica de habilitação de leitura
    always @(address) begin
        if (address >= 0 && address <= 127)
            EN = 1'b1;
        else
            EN = 1'b0;
    end

    // Lógica de saída dos dados
    always @(posedge clock) begin
        if (EN)
            data_out = prmemo[address];
       // if (write_enable && EN)
         //   prmemo[address] = data_in;  // Se habilitado para escrita, escreve os dados na memória
    end

endmodule
	
//DATA MEMORY
module rw_48x8_sync ( // essa é a data memory, é uma memória normal. Aparentemente, serve mais para ajudar a fazer contas maiores e coisinhas desse tipo de suporte ao CPU 
    input wire [7:0] address,  // Endereço da RAM
    input wire clock,          // Clock
    input wire write,          // Sinal de escrita
    input wire [7:0] data_in,  // Dados de entrada para escrita
    output reg [7:0] data_out  // Dados de saída para leitura
);
  reg[7:0] RW[128:175];  // Memória RAM de 96 endereços, 8 bits cada(é uma continuação do passado, por isso começa do 128)
  integer EN;

  always @ (address) // verifica se o endereço fornecido está dentro dos limites da data memory
		begin
          if ( (address >= 128) && (address <= 175) )
				EN = 1'b1;
			else
				EN = 1'b0;
		end
    
  always @ (posedge clock) // verifica se o endereço é coerente antes de escrever ou enviar os dados
		begin
			if (write && EN)
              RW[address] = data_in;
			else if (!write && EN)
				data_out = RW[address];
		end
endmodule


//PILHA
module pilha ( // essa é a data memory, é uma memória normal. Aparentemente, serve mais para ajudar a fazer contas maiores e coisinhas desse tipo de suporte ao CPU 
    input wire [7:0] address,  // Endereço da RAM
    input wire clock,          // Clock
    input wire write,          // Sinal de escrita
    input wire [7:0] data_in,  // Dados de entrada para escrita
    output reg [7:0] data_out  // Dados de saída para leitura
);
  reg[7:0] pilha_mem[176:223];  // Memória RAM de 96 endereços, 8 bits cada(é uma continuação do passado, por isso começa do 176)
  integer EN;
  
  always @ (address) // verifica se o endereço fornecido está dentro dos limites da data memory
		begin
          if ( (address >= 176) && (address <= 223) )
				EN = 1'b1;
			else
				EN = 1'b0;
		end
    
  always @ (posedge clock) // verifica se o endereço é coerente antes de escrever ou enviar os dados
		begin
			if (write && EN)
              pilha_mem[address] = data_in;
			else if (!write && EN)
				data_out = pilha_mem[address];
		end
endmodule


//MEMORY de fato
module memory (
    input wire [7:0] address,           // Endereço de memória (8 bits)
    input wire [7:0] data_in,           // Dados de entrada para escrita
    input wire write,                   // Habilitação de escrita
    input wire clock,                   // Sinal de clock
    input wire reset,                   // Reset do sistema
    input wire [7:0] port_in [0:15],    // 16 portas de entrada, 8 bits cada
    output reg [7:0] data_out,          // Saída de dados principal
    output reg [7:0] port_out [0:15]    // 16 portas de saída, 8 bits cada
);

    wire [7:0] rom_data_out;  // Saída da ROM
    wire [7:0] ram_data_out;  // Saída da RAM
    wire [7:0] pilha_data_out; // saida da pilha

    // Instanciação da PROGRAM MEMORY
    programmemory prmemo (
        .address(address),
        .clock(clock),
        .data_out(rom_data_out)
    );

    // Instanciação da RAM
    rw_48x8_sync ram_inst (
        .address(address),
        .clock(clock),
        .write(write),
        .data_in(data_in),
        .data_out(ram_data_out)
    );
  
    // Instanciação da PILHA
    pilha pilha(
        .address(address),
        .clock(clock),
        .write(write),
        .data_in(data_in),
        .data_out(pilha_data_out)
    );
    

  //vou ficar faltando com uma informação precisa de como essa parte funciona, dps verifiquem na pag 20 do arquivo ou 162 do livro a explicação. Aparentemente, isso é para pegar a informação do data_in e jogar para a porta de saída selecionada pelo endereço.O !reset é para verificar se porta pode ta funcionando e o write se tá podendo receber as informações do data_in. O <= significa atribuição não bloqueada, pelo oq eu entendi siginifica q todas as portas vão receber o valor ao mesmo tempo, n sendo uma atribuição imediata como no "=", para ser algo mais semelhante com os flip flops na vida real na qual só é atualizado quando passa o ciclo do clock
  // Bloco combinando toda a lógica de endereçamento
// Bloco para gerenciamento das portas de saída
    integer i;
    always @(posedge clock or negedge reset) begin
        if (!reset) begin
            // Resetando todas as portas de saída
            for (i = 0; i < 16; i = i + 1) begin
                port_out[i] <= 8'h00;
            end
        end else if (write && (address >= 8'hE0 && address <= 8'hEF)) begin
            port_out[address - 8'hE0] <= data_in; // Escrevendo na porta de saída correspondente
        end
    end

    // Multiplexador para selecionar a saída apropriada
    always @(*) begin
        if (address <= 8'd127) 
            data_out = rom_data_out; // ROM
        else if (address <= 8'd223) 
            data_out = ram_data_out; // RAM
        else if (address >= 8'hF0 && address <= 8'hFF) 
            data_out = port_in[address - 8'hF0]; // Portas de entrada
        else 
            data_out = 8'h00; // Valor padrão
    end
endmodule
//***************************************************************

//CPU.V

// Portas lógicas
module and_gate (output reg [7:0] Y, input [7:0] A, B);
    integer i;
    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            if (A[i] == 1 && B[i] == 1)
                Y[i] = 1;
            else
                Y[i] = 0;
        end
    end
endmodule

//OR
module or_gate (output reg [7:0] Y, input [7:0] A, B);
    integer i;
    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            if (A[i] == 1 || B[i] == 1)
                Y[i] = 1;
            else
                Y[i] = 0;
        end
    end
endmodule

//XOR
module xor_gate (output reg [7:0] Y, input [7:0] A, B);
    integer i;
    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            if (A[i] != B[i])
                Y[i] = 1;
            else
                Y[i] = 0;
        end
    end
endmodule

//NAND
module nand_gate (output reg [7:0] Y, input [7:0] A, B);
    integer i;
    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            if (A[i] == 1 && B[i] == 1)
                Y[i] = 0;
            else
                Y[i] = 1;
        end
    end
endmodule

//NOR
module nor_gate (output reg [7:0] Y, input [7:0] A, B);
    integer i;
    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            if (A[i] == 0 && B[i] == 0)
                Y[i] = 1;
            else
                Y[i] = 0;
        end
    end
endmodule

//XNOR
module xnor_gate (output reg [7:0] Y, input [7:0] A, B);
    integer i;
    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            if (A[i] == B[i])
                Y[i] = 1;
            else
                Y[i] = 0;
        end
    end
endmodule

//NOT
module not_gate (output reg [7:0] Y, input [7:0] A);
    integer i;
    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            if (A[i] == 1)
                Y[i] = 0;
            else
                Y[i] = 1;
        end
    end
endmodule

//-------- Deslocamento e rotação de bits----------
module BitShiftRotate (
  input [7:0] reg_in, // registrador de entrada
  input shift_enable, // habilita o deslocamento
  input shift_dir, // 110 esquerda 111 direita
  input rotate_enable, // habilita a rotação
  input rotate_dir, // 0 esquerda 1 direita
  output reg [7:0] reg_out, // registrador de entrada
  output reg carry_flag // flag de carry (bit perdido)
);
  
  always @(*) begin
    if (shift_enable) begin
      if (shift_dir == 3'b110) begin
        reg_out = reg_in << 1; 
        carry_flag = reg_in[`WORD_SIZE-1]; // bit mais significativo
      end else if (shift_dir == 3'b111) begin
        reg_out = reg_in >> 1;
        carry_flag = reg_in[0]; // bit menos significativo
      end
      
    end else if (rotate_enable) begin
      if (rotate_dir == 0) begin
        reg_out = {reg_in[`WORD_SIZE-2:0], reg_in[`WORD_SIZE-1]}; // {..., ...} concatena as duas partes
        carry_flag = reg_in[`WORD_SIZE-1]; // bit mais significativo vai para carry
      end else begin
        reg_out = {reg_in[0], reg_in[`WORD_SIZE-1:1]};
        carry_flag = reg_in[0]; // bit menos significativo vai para carry
      end
      
    end else begin
      reg_out = reg_in;
      carry_flag = 0;      
    end
  end
endmodule

//ALU    
module alu( // n alterei nada na ula enviada pelo gpt
    input wire [7:0] A,         // Operando A
    input wire [7:0] B,         // Operando B
    input wire [3:0] ALU_Sel,   // Selector da operação
    output reg [7:0] ALU_Out,   // Resultado da ALU
    output reg [3:0] NZVC          // Vetor de Flags
);

    wire [7:0] and_result, or_result, xor_result, nand_result, nor_result, xnor_result, not_result;
    // instanciando as portas lógicas
    and_gate and_inst(.Y(and_result), .A(A[7:0]), .B(B[7:0]));
    or_gate or_inst(.Y(or_result), .A(A[7:0]), .B(B[7:0]));
    xor_gate xor_inst(.Y(xor_result), .A(A[7:0]), .B(B[7:0]));
    nand_gate nand_inst(.Y(nand_result), .A(A[7:0]), .B(B[7:0]));
    nor_gate nor_inst(.Y(nor_result), .A(A[7:0]), .B(B[7:0]));
    xnor_gate xnor_inst(.Y(xnor_result), .A(A[7:0]), .B(B[7:0]));
    not_gate not_inst(.Y(not_result), .A(A[7:0]));

    always @(*) begin
        // Operações baseadas no seletor
        case (ALU_Sel)
            3'b000: ALU_Out = A + B;          // Soma
            3'b001: ALU_Out = A - B;          // Subtração
            3'b010: ALU_Out = A & B;          // AND
            3'b011: ALU_Out = A | B;          // OR
            3'b100: ALU_Out = A ^ B;          // XOR
            3'b101: ALU_Out = ~A;             // NOT (Somente A)
            3'b110: ALU_Out = A << 1;         // Shift lógico para a esquerda
            3'b111: ALU_Out = A >> 1;         // Shift lógico para a direita
            default: ALU_Out = 8'b00000000;   // Valor padrão
        endcase

        /*// Atualização das flags
        Zero     = (ALU_Out == 8'b0);         // Flag Zero
        Negative = ALU_Out[7];                // Flag Negativo (MSB = 1)
        Carry    = (A + B > 8'b11111111);     // Carry gerado na soma
        Overflow = ((A[7] & B[7] & ~ALU_Out[7]) | (~A[7] & ~B[7] & ALU_Out[7])); // Overflow*/
    end
endmodule
   
// ----------SetRegist--------
module SetBit (
  input [`WORD_SIZE-1:0] reg_in,
  input [3:0] pos,
  output [`WORD_SIZE-1:0] reg_out
);

  wire [`WORD_SIZE-1:0] mask;
  assign mask = (1 << pos); // Gera máscara com 1 na posição desejada
  assign reg_out = reg_in | mask; // Usa OR para setar o bit em 1

endmodule


//---------- ClearBit---------
module ClearBit (
  input [`WORD_SIZE-1:0] reg_in,
  input [3:0] pos,
  output [`WORD_SIZE-1:0] reg_out
);
  
  wire [`WORD_SIZE-1:0] mask;
  assign mask = ~(1 << pos);
  assign reg_out = reg_in & mask;
  
endmodule

//-------------------- ALU + módulos(operações lógicas/manioulação bits) LEGALIZADA -----------------------
// AND Gate
module and_gate (output reg [7:0] Y, input [7:0] A, B);
    always @(*) begin
        Y = A & B; // Operação vetorial
    end
endmodule

// OR Gate
module or_gate (output reg [7:0] Y, input [7:0] A, B);
    always @(*) begin
        Y = A | B; // Operação vetorial
    end
endmodule

// XOR Gate
module xor_gate (output reg [7:0] Y, input [7:0] A, B);
    always @(*) begin
        Y = A ^ B; // Operação vetorial
    end
endmodule

// NAND Gate
module nand_gate (output reg [7:0] Y, input [7:0] A, B);
    always @(*) begin
        Y = ~(A & B); // Operação vetorial
    end
endmodule

// NOR Gate
module nor_gate (output reg [7:0] Y, input [7:0] A, B);
    always @(*) begin
        Y = ~(A | B); // Operação vetorial
    end
endmodule

// XNOR Gate
module xnor_gate (output reg [7:0] Y, input [7:0] A, B);
    always @(*) begin
        Y = ~(A ^ B); // Operação vetorial
    end
endmodule

// NOT Gate
module not_gate (output reg [7:0] Y, input [7:0] A);
    always @(*) begin
        Y = ~A; // Operação vetorial
    end
endmodule

// Bit Shift and Rotate
module BitShiftRotate (
    input [7:0] reg_in, 
    input shift_enable,
    input shift_dir, 
    input rotate_enable,
    input rotate_dir,
    output reg [7:0] reg_out,
    output reg carry_flag
);
    always @(*) begin
        reg_out = reg_in;
        carry_flag = 0;

        if (shift_enable) begin
            if (shift_dir == 0) begin // Shift left
                reg_out = reg_in << 1;
                carry_flag = reg_in[7];
            end else begin // Shift right
                reg_out = reg_in >> 1;
                carry_flag = reg_in[0];
            end
        end else if (rotate_enable) begin
            if (rotate_dir == 0) begin // Rotate left
                reg_out = {reg_in[6:0], reg_in[7]};
                carry_flag = reg_in[7];
            end else begin // Rotate right
                reg_out = {reg_in[0], reg_in[7:1]};
                carry_flag = reg_in[0];
            end
        end
    end
endmodule

module alu( 
    input wire [7:0] A,         // Operando A
    input wire [7:0] B,         // Operando B
    input wire [2:0] ALU_Sel,   // Selector da operação
    output reg [7:0] ALU_Out,   // Resultado da ALU
    output reg [3:0] NZVC       // Vetor de flags (N, Z, V, C)
);

    // Fios para armazenar saídas dos módulos lógicos
    wire [7:0] and_result, or_result, xor_result, nand_result, nor_result, xnor_result, not_result;

    // Instanciando as portas lógicas
    and_gate u_and(.Y(and_result), .A(A), .B(B));
    or_gate u_or(.Y(or_result), .A(A), .B(B));
    xor_gate u_xor(.Y(xor_result), .A(A), .B(B));
    nand_gate u_nand(.Y(nand_result), .A(A), .B(B));
    nor_gate u_nor(.Y(nor_result), .A(A), .B(B));
    xnor_gate u_xnor(.Y(xnor_result), .A(A), .B(B));
    not_gate u_not(.Y(not_result), .A(A));

    wire [7:0] shift_rotate_out;
    wire carry_flag;

    // Instanciando o módulo de deslocamento e rotação
    BitShiftRotate u_shift_rotate(
        .reg_in(A),
        .shift_enable((ALU_Sel == 3'b110 || ALU_Sel == 3'b111)), // Shift enable
        .shift_dir(ALU_Sel[0]),  // Direção do shift: 0 para esquerda, 1 para direita
        .rotate_enable(1'b0),   // Rotação desabilitada
        .rotate_dir(1'b0),      // Não se aplica
        .reg_out(shift_rotate_out),
        .carry_flag(carry_flag)
    );

    always @(*) begin
        // Inicializa flags
        NZVC = 4'b0000;

        // Operações baseadas no seletor
        case (ALU_Sel)
            3'b000: begin // Soma
                {NZVC[0], ALU_Out} = A + B; // Carry
                NZVC[3] = ALU_Out[7];       // Negativo
                NZVC[2] = (ALU_Out == 8'b0); // Zero
                NZVC[1] = (~A[7] & ~B[7] & ALU_Out[7]) | (A[7] & B[7] & ~ALU_Out[7]); // Overflow
            end
            3'b001: begin // Subtração
                {NZVC[0], ALU_Out} = A - B; // Borrow
                NZVC[3] = ALU_Out[7];       // Negativo
                NZVC[2] = (ALU_Out == 8'b0); // Zero
                NZVC[1] = (A[7] & ~B[7] & ~ALU_Out[7]) | (~A[7] & B[7] & ALU_Out[7]); // Overflow
            end
            3'b010: begin // AND
                ALU_Out = and_result;       
                NZVC[3] = ALU_Out[7];       // Negativo
                NZVC[2] = (ALU_Out == 8'b0); // Zero
            end
            3'b011: begin // OR
                ALU_Out = or_result;        
                NZVC[3] = ALU_Out[7];       // Negativo
                NZVC[2] = (ALU_Out == 8'b0); // Zero
            end
            3'b100: begin // XOR
                ALU_Out = xor_result;       // Usando saída do módulo `xor_gate`
                NZVC[3] = ALU_Out[7];       // Negativo
                NZVC[2] = (ALU_Out == 8'b0); // Zero
            end
            3'b101: begin // NOT
                ALU_Out = not_result;       // Usando saída do módulo `not_gate`
                NZVC[3] = ALU_Out[7];       // Negativo
                NZVC[2] = (ALU_Out == 8'b0); // Zero
            end
            3'b110: begin // Shift lógico para a esquerda
                ALU_Out = shift_rotate_out; 
                NZVC[3] = ALU_Out[7];       // Negativo
                NZVC[2] = (ALU_Out == 8'b0); // Zero
                NZVC[0] = carry_flag;       // Carry
            end
            3'b111: begin // Shift lógico para a direita
                ALU_Out = shift_rotate_out; 
                NZVC[3] = ALU_Out[7];       // Negativo
                NZVC[2] = (ALU_Out == 8'b0); // Zero
                NZVC[0] = carry_flag;       // Carry
            end
            default: ALU_Out = 8'b00000000;
        endcase
    end
endmodule

// ---------------------------------------------------------------------

    
//DATA_PATH    
module data_path (
    input wire clock,
    input wire reset,
    input wire [2:0] Bus1_Sel,
    input wire [1:0] Bus2_Sel,
    input wire [3:0] ALU_Sel,
    input wire IR_Load,
    input wire MAR_Load,
    input wire PC_Load,
    input wire PC_Inc,
    input wire A_Load,
    input wire B_Load,
  	input wire C_Load,
    input wire TEMP_Load,
    input wire CCR_Load,
    input wire [7:0] from_memory,
    output reg [7:0] to_memory,
    output reg [7:0] address,
    output reg [7:0] IR_Out, 
    output wire [3:0] CCR_Out, 
    input wire T_Dec,
    input wire T_Inc,
    input wire Ads_Sel

);

    // Registradores
  	reg [7:0] IR, MAR, PC, A, B, C, TEMP, CCR, T;
    reg [7:0] Bus1, Bus2, ALU_Result, NZVC;
  
  	
  //Multiplexadores(pag 23)
  always @ (Bus1_Sel, PC, A, B, C, TEMP, MAR)
		begin: MUX_BUS1
			case (Bus1_Sel)
				3'b000 : Bus1 = PC;
				3'b001 : Bus1 = A;
				3'b010 : Bus1 = B;
        3'b011 : Bus1 = C;
        3'b100 : Bus1 = MAR + 1;
        3'b101 : Bus1 = TEMP;
				default : Bus1 = 8'hXX;
			endcase
		end
	always @ (Bus2_Sel, ALU_Result, Bus1, from_memory)
		begin: MUX_BUS2
			case (Bus2_Sel)
				2'b00 : Bus2 = ALU_Result;
				2'b01 : Bus2 = Bus1;
				2'b10 : Bus2 = from_memory;
				default : Bus1 = 8'hXX;
			endcase
		end
	always @ (Bus1, IR)
		begin
			to_memory = Bus1;
			IR_Out = IR;
		end
  always @ (Ads_Sel, MAR , T)
		begin: MUX_Address
			case (Ads_Sel)
				1'b0 : address = MAR;
				1'b1 : address = T;
				default : address = MAR;
			endcase
		end
  
  //Resgistradores:
  //pag 23(atribuição de valor do IR)
    always @ (posedge clock or negedge reset)
		begin: INSTRUCTION_REGISTER
			if (!reset)
				IR <= 8'h00;
			
			if (IR_Load)
				IR = Bus2;
	end
  //pag 23(do MAR dessa vez)
  	always @ (posedge clock or negedge reset)
		begin: MEMORY_ADDRESS_REGISTER
			if (reset != 0)
				MAR <= 8'h00;
			
			if (MAR_Load==1)
				MAR = Bus2;
	end
  //pag 24(PC- tem um incremento dessa vez ao invés de só receber informação)
	always @ (posedge clock or negedge reset)
		begin: PROGRAM_COUNTER
			if (reset != 0)
				PC <= 8'h00;
			
			if (PC_Load == 1)
				PC <= Bus2;
			 if (PC_Inc == 1)
				PC = MAR + 1;
	end
  //pag 24(registradores gerais)
  	//registrador a
	always @ (posedge clock or negedge reset)
		begin: A_REGISTER
			if (reset != 0)
				A <= 8'h00;
			
			if (A_Load == 1)
				A = Bus2;
	end         
	//registrador b
    always @ (posedge clock or negedge reset)
		begin: B_REGISTER
			if (reset != 0)
				B <= 8'h00;
			
			if (B_Load == 1)
				B = Bus2;
	end
  	//registrador c
    always @ (posedge clock or negedge reset)
		begin: C_REGISTER
			if (reset != 0)
				C <= 8'h00;
			 if (C_Load == 1)
				C = Bus2;
	end
  //registrador temp
    always @ (posedge clock or negedge reset)
		begin: TEMP_REGISTER
			if (reset != 0)
				TEMP <= 8'h00;
			 if (TEMP_Load == 1)
				TEMP = Bus2;
	end
  //pag 24(CCR)
	always @ (posedge clock or negedge reset)
		begin: CONDITION_CODE_REGISTER
			if (reset != 0)
				CCR <= 8'h00;
			
			if (CCR_Load == 1)
				CCR = NZVC;
	end
  //Topo da pilha
  always @ (posedge clock or negedge reset)
		begin: TOPO
			if (reset != 0)
				T = 8'hAF;
			
			if (T_Dec == 1)
			  T = T - 1 ;
			 if (T_Inc == 1)
				T = T + 1;
	end
  
    

// Instância do módulo ALU
    alu alu_instance (
        .A(A),
        .B(B),
        .ALU_Sel(ALU_Sel)
        /*.ALU_Out(ALU_Out),
        .Zero(Zero),
        .Negative(Negative),
        .Carry(Carry),
        .Overflow(Overflow)*/
    );


endmodule


//UNIDADE DE CONTROLE    
module control_unit (
    input wire clock,
    input wire reset,
    input wire [7:0] IR,
    input wire [3:0] CCR,
    output reg write,
    output reg IR_Load,
    output reg MAR_Load,
    output reg PC_Load,
    output reg PC_Inc,
    output reg A_Load,
    output reg B_Load,
  	output reg C_Load,
    output reg TEMP_Load,
    output reg CCR_Load,
    output reg [3:0] ALU_Sel,
    output reg Bus1_Sel,
    output reg Bus2_Sel,
    output reg Ads_Sel,
    output reg T_Dec,
    output reg T_Inc
);

  	//FSM
  	//DEFINICAO DOS PARAMETROS
    reg [7:0] current_state, next_state;
	parameter S_FETCH_0 = 0, //-- Opcode fetch states
          S_FETCH_1 = 1,
          S_FETCH_2 = 2,
          S_DECODE_3 = 3, //-- Opcode decode state
          S_LDA_IMM_4 = 4, //-- Load A (Immediate) states
          S_LDA_IMM_5 = 5,
          S_LDA_IMM_6 = 6,
          S_LDA_DIR_4 = 7, //-- Load A (Direct) states
          S_LDA_DIR_5 = 8,
          S_LDA_DIR_6 = 9,
          S_LDA_DIR_7 = 10,
          S_LDA_DIR_8 = 11,
          S_STA_DIR_4 = 12, //-- Store A (Direct) States
          S_STA_DIR_5 = 13,
          S_STA_DIR_6 = 14,
          S_STA_DIR_7 = 15,
          S_LDB_IMM_4 = 16, //-- Load B (Immediate) states
          S_LDB_IMM_5 = 17,
          S_LDB_IMM_6 = 18,
          S_LDB_DIR_4 = 19, //-- Load B (Direct) states
          S_LDB_DIR_5 = 20,
          S_LDB_DIR_6 = 21,
          S_LDB_DIR_7 = 22,
          S_LDB_DIR_8 = 23,
          S_STB_DIR_4 = 24, //-- Store B (Direct) States
          S_STB_DIR_5 = 25,
          S_STB_DIR_6 = 26,
          S_STB_DIR_7 = 27,
  		    S_LDC_IMM_4 = 28, //-- Load C (Immediate) states
          S_LDC_IMM_5 = 29,
          S_LDC_IMM_6 = 30,
          S_LDC_DIR_4 = 31, //-- Load C (Direct) states
          S_LDC_DIR_5 = 32,
          S_LDC_DIR_6 = 33,
          S_LDC_DIR_7 = 34,
          S_LDC_DIR_8 = 35,
          S_STC_DIR_4 = 36, //-- Store C (Direct) States
          S_STC_DIR_5 = 37,
          S_STC_DIR_6 = 38,
          S_STC_DIR_7 = 39,
          S_BRA_4 = 40, //-- Branch Always States
          S_BRA_5 = 41,
          S_BRA_6 = 42,
          S_BEQ_4 = 43, //-- Branch if Equal Zero States
          S_BEQ_5 = 44,
          S_BEQ_6 = 45,
          S_BEQ_7 = 46,
          S_BNQ_4 = 47, //-- Branch if Not Equal To Zero
          S_BNQ_5 = 48,
          S_BNQ_6 = 49,
          S_BNQ_7 = 50,
          S_BGT_4 = 51, //-- Branch if Greater Than
          S_BGT_5 = 52,
          S_BGT_6 = 53, 
          S_BGT_7 = 54,
          S_BLT_4 = 55, //-- Branch if Less Than
          S_BLT_5 = 56,
          S_BLT_6 = 57,
          S_BLT_7 = 58,
          S_CALL_4 = 59, //-- Call Always
          S_CALL_5 = 60,
          S_CALL_6 = 61,
          S_CALL_7 = 62,
          S_RET_4 = 63, //-- RETORNAR
          S_RET_5 = 64,
          S_RET_6 = 65,
          S_ADD_AB_4 = 66; //-- Addition States
          S_SUB_AB_4 = 67; //-- Subtração
          S_MUL_AB_4 = 68; //--  Multiplicação
          S_DIV_AB_4 = 69; //-- Divisão
          S_MOD_AB_4 = 70; //-- Resto da divisão
          S_NOT_4 = 72;
          S_AND_4 = 73;
          S_NAND_4 = 74;
          S_OR_4 = 75;
          S_NOR_4 = 76;
          S_XOR_4 = 77;
          S_XNOR_4 = 78;
          S_SHL_A_4 = 79;
          S_SHL_B_4 = 80;
          S_SHR_A_4 = 81;
          S_SHR_B_4 = 82;
          S_CPE_AB_4 = 83; //-- Comparador
          S_CPG_AB_4 = 84; //-- Comparador
          S_CPL_AB_4 = 85; //-- Comparador
          S_PUSH_4 = 86;
          S_PUSH_5 = 87; 
          S_PUSH_6 = 88; 
          S_PUSH_7 = 89;  
          S_POP_4 = 90;
          S_POP_5 = 91;
          S_POP_6 = 92;
          S_CMP_4 = 93;
          S_NOP_4 = 94;
          S_IN_4 = 95;
          S_OUT_4 = 9;
          

	//ESTADO DE MEMORIA
    always @ (posedge clock or negedge reset)
        begin: STATE_MEMORY
            if (reset != 0)
                current_state <= S_FETCH_0;
            else
                current_state <= next_state;
    end

	//LOGICA DO PROXIMO ESTADO
    always @ (current_state, IR, CCR)
        begin: NEXT_STATE_LOGIC
            case (current_state)
                S_FETCH_0 : next_state = S_FETCH_1; // Path for FETCH instruction
                S_FETCH_1 : next_state = S_FETCH_2;
                S_FETCH_2 : next_state = S_DECODE_3;
                S_DECODE_3 : 
                if (IR == 11) // -> LDA_IMM 
                        next_state = S_LDA_IMM_4; // Load A (Immediate)
              	else if (IR == 12) // -> LDA
                        next_state = S_LDA_DIR_4; // Load A (Direct)
             	 else if (IR == 13) 
                        next_state = S_STA_DIR_4; // Store A (Direct)
             	 else if (IR == 14) 
                        next_state = S_LDB_IMM_4; // Load B (Immediate)
              	else if (IR == 15) 
                        next_state = S_LDB_DIR_4; // Load B (Direct)
              	else if (IR == 16) 
                        next_state = S_STB_DIR_4; // Store B (Direct) 
                else if (IR == 14) 
                        next_state = S_LDC_IMM_4; // Load C (Immediate)
              	else if (IR == 15) 
                        next_state = S_LDC_DIR_4; // Load C (Direct)
              	else if (IR == 16) 
                        next_state = S_STC_DIR_4; // Store C (Direct) 
              	else if (IR == 17) 
                        next_state = S_BRA_4; // Branch 
              	else if(IR == 18 && CCR[2] == 0) // Z = 0 -> significa que a condicional de BEQ não foi atendida(Z=0)
                        next_state = S_BEQ_7;
              	else if(IR == 18 && CCR[2] == 1) // Z é o segundo bit mais significativo de CCR -> significa que a condicional de BEQ foi atendida(Z=1)
                        next_state = S_BEQ_4;
             	 else if(IR == 19 && CCR[2] == 0) // Z = 0, significa que a condicional de BNQ foi atendida
                        next_state = S_BNQ_4;
              	else if(IR == 19 && CCR[2] == 1) // Z = 1, significa que a condicional de BNQ não foi atendida
                        next_state = S_BNQ_7;
             	 else if(IR == 20 && CCR[3] == 0 && CCR[1] == 0 && CCR[2] == 0) // N = 0 = V, Z = 0 significa que a condicional de BGT foi atendida
                        next_state = S_BGT_4;
             	 else if(IR == 20 && CCR[3] == 1 && CCR[1] == 0 && CCR[2] == 0) // N = 1, Z = 0 significa que a condicional de BGT não foi atendida
                        next_state = S_BGT_7;
              	else if(IR == 21 && CCR[3] == 1 && CCR[1] == 0 && CCR[2] == 0) // significa que a condicional de BLT foi atendida
                        next_state = S_BLT_4;
             	 else if(IR == 21 && CCR[3] == 0 && CCR[1] == 0 && CCR[2] == 0) // significa que a condicional de BLT não foi atendida
                        next_state = S_BLT_7;
              	else if(IR == 22) // call
                        next_state = S_CALL_4;
                else if(IR == 23) // ret
                        next_state = S_RET_4;
              	else if (IR == 24) //add 
                        next_state = S_ADD_AB_4; // Add A and B
                else if (IR == 25) //sub
                        next_state = S_SUB_AB_4; // sub A and B
                else if (IR == 26) //mult 
                        next_state = S_MUL_AB_4; // mult A and B
                else if (IR == 27) //div
                        next_state = S_DIV_AB_4; // div A and B
                else if (IR == 28) //mod
                        next_state = S_MOD_AB_4; // mod A and B
                else if (IR == 29) //cp
                        next_state = S_CPE_AB_4; // comp A and B
                else if (IR == 30) //cp
                        next_state = S_CPG_AB_4; // comp A and B
                else if (IR == 31) //cp
                        next_state = S_CPL_AB_4; // comp A and B
                else if (IR == 32) //not
                        next_state = S_NOT_4;
                else if (IR == 33) //and
                        next_state = S_AND_4;
                else if (IR == 34) //nand
                        next_state = S_NAND_4;
                else if (IR == 35) //or
                        next_state = S_OR_4;
                else if (IR == 36) //nor
                        next_state = S_NOR_4;
                else if (IR == 37) //xor
                        next_state = S_XOR_4;
                else if (IR == 38) //xnor
                        next_state = S_XNOR_4;
                else if (IR == 39) //
                        next_state = S_SHL_A_4;
                else if (IR == 40) //
                        next_state = S_SHL_B_4;
                else if (IR == 41) //
                        next_state = S_SHR_A_4;
                else if (IR == 42) //
                        next_state = S_SHR_B_4;
                else if (IR == 43) //
                        next_state = S_CMP_4;
                else if (IR == 44) //
                        next_state = S_INC_A_4;
                else if (IR == 45) //
                        next_state = S_INC_B_4;
                else if (IR == 46) //
                        next_state = S_DEC_A_4;
                else if (IR == 47) //
                        next_state = S_DEC_B_4;
                else if (IR == 48) //
                        next_state = S_PUSH_4;
                else if (IR == 49) //
                        next_state = S_POP_4;
                else if (IR == 50) //
                        next_state = S_NOP_4;
                else 
                        next_state = S_FETCH_0; // Default fallback
                S_LDA_IMM_4 : next_state = S_LDA_IMM_5; // Path for LDA_IMM instruction
                S_LDA_IMM_5 : next_state = S_LDA_IMM_6;
                S_LDA_IMM_6 : next_state = S_FETCH_0;
              	S_LDB_IMM_4 : next_state = S_LDB_IMM_5; // Path for LDB_IMM instruction
                S_LDB_IMM_5 : next_state = S_LDB_IMM_6;
                S_LDB_IMM_6 : next_state = S_FETCH_0;
              	S_LDC_IMM_4 : next_state = S_LDC_IMM_5; // Path for LDC_IMM instruction
                S_LDC_IMM_5 : next_state = S_LDC_IMM_6;
                S_LDC_IMM_6 : next_state = S_FETCH_0;
              	//LD*_DIR
              	S_LDA_DIR_4 : next_state = S_LDA_DIR_5; // Path for LDA_DIR instruction
                S_LDA_DIR_5 : next_state = S_LDA_DIR_6;
                S_LDA_DIR_6 : next_state = S_LDA_DIR_7;
              	S_LDA_DIR_7 : next_state = S_LDA_DIR_8;
              	S_LDA_DIR_8 : next_state = S_FETCH_0;
              	S_LDB_DIR_4 : next_state = S_LDB_DIR_5; // Path for LDB_DIR instruction
                S_LDB_DIR_5 : next_state = S_LDB_DIR_6;
                S_LDB_DIR_5 : next_state = S_LDB_DIR_6;
                S_LDB_DIR_6 : next_state = S_LDB_DIR_7;
              	S_LDB_DIR_7 : next_state = S_LDB_DIR_8;
              	S_LDB_DIR_8 : next_state = S_FETCH_0;
              	S_LDC_DIR_4 : next_state = S_LDC_DIR_5; // Path for LDC_DIR instruction
                S_LDC_DIR_5 : next_state = S_LDC_DIR_6;
                S_LDC_DIR_5 : next_state = S_LDC_DIR_6;
                S_LDC_DIR_6 : next_state = S_LDC_DIR_7;
              	S_LDC_DIR_7 : next_state = S_LDC_DIR_8;
              	S_LDC_DIR_8 : next_state = S_FETCH_0;
              	//ST*_DIR
              	S_STA_DIR_4 : next_state = S_STA_DIR_5; // Path for STA_DIR instruction
                S_STA_DIR_5 : next_state = S_STA_DIR_6;
                S_STA_DIR_6 : next_state = S_STA_DIR_7;
              	S_STA_DIR_7 : next_state = S_FETCH_0;
              	S_STB_DIR_4 : next_state = S_STB_DIR_5; // Path for STB_DIR instruction
                S_STB_DIR_5 : next_state = S_STB_DIR_6;
                S_STB_DIR_5 : next_state = S_STB_DIR_6;
                S_STB_DIR_6 : next_state = S_STB_DIR_7;
              	S_STB_DIR_7 : next_state = S_FETCH_0;
              	S_STC_DIR_4 : next_state = S_STC_DIR_5; // Path for STC_DIR instruction
                S_STC_DIR_5 : next_state = S_STC_DIR_6;
                S_STC_DIR_5 : next_state = S_STC_DIR_6;
                S_STC_DIR_6 : next_state = S_STC_DIR_7;
              	S_STC_DIR_7 : next_state = S_FETCH_0;
              	//BRA(SEM CONDICIONAL)
              	S_BRA_4 : next_state = S_BRA_5; // Path for BRA instruction
                S_BRA_5 : next_state = S_BRA_6;
                S_BRA_6 : next_state = S_FETCH_0;
              	//BEQ(COM CONDICIONAL)
              	S_BEQ_4 : next_state = S_BEQ_5; // Path for BEQ instruction(caso dê bom)
                S_BEQ_5 : next_state = S_BEQ_6;
                S_BEQ_6 : next_state = S_FETCH_0;
              	S_BEQ_7 : next_state = S_FETCH_0; // Path for BEQ instruction(caso dê ruim)
                //BNQ
                S_BNQ_4 : next_state = S_BNQ_5; // Path for BNQ instruction(caso dê bom)
                S_BNQ_5 : next_state = S_BNQ_6;
                S_BNQ_6 : next_state = S_FETCH_0;
              	S_BNQ_7 : next_state = S_FETCH_0; // Path for BNQ instruction(caso dê ruim)
                //BGT
                S_BGT_4 : next_state = S_BGT_5; // Path for BGT instruction(caso dê bom)
                S_BGT_5 : next_state = S_BGT_6;
                S_BGT_6 : next_state = S_FETCH_0;
              	S_BGT_7 : next_state = S_FETCH_0; // Path for BGT instruction(caso dê ruim)
                //BLT
                S_BLT_4 : next_state = S_BLT_5; // Path for BLT instruction(caso dê bom)
                S_BLT_5 : next_state = S_BLT_6;
                S_BLT_6 : next_state = S_FETCH_0;
              	S_BLT_7 : next_state = S_FETCH_0; // Path for BLT instruction(caso dê ruim)
                //CALL
                S_CALL_4 : next_state = S_CALL_5; // Path for CALL instruction
                S_CALL_5 : next_state = S_CALL_6;
                S_CALL_6 : next_state = S_CALL_7;
                S_CALL_7 : next_state = S_FETCH_0;
                //RET
                S_RET_4 : next_state = S_RET_5; // Path for RET instruction
                S_RET_5 : next_state = S_RET_6;
                S_RET_6 : next_state = S_FETCH_0;
                //OPERAÇÕES ULA
                //ADD
              	S_ADD_AB_4 : next_state = S_FETCH_0; // Path for ADD_AB instruction
                //SUB
              	S_SUB_AB_4 : next_state = S_FETCH_0; // Path for SUB_AB instruction
                //MULT
              	S_MUL_AB_4 : next_state = S_FETCH_0; // Path for MUL_AB instruction
                //DIV
              	S_DIV_AB_4 : next_state = S_FETCH_0; // Path for DIV_AB instruction
                //MOD
              	S_MOD_AB_4 : next_state = S_FETCH_0; // Path for MOD_AB instruction
                //CP
              	S_CMP_AB_4 : next_state = S_FETCH_0; 
                //NOT
              	S_NOT_4 : next_state = S_FETCH_0; 
                //AND
              	S_AND_4 : next_state = S_FETCH_0; 
                //NAND
              	S_NAND_4 : next_state = S_FETCH_0; 
                //OR
              	S_OR_4 : next_state = S_FETCH_0; 
                //NOR
              	S_NOR_4 : next_state = S_FETCH_0; 
                //XOR
              	S_XOR_4 : next_state = S_FETCH_0; 
                //XNOR
              	S_XNOR_4 : next_state = S_FETCH_0;
                S_CPE_AB_4 : next_state = S_FETCH_0;
                S_CPG_AB_4 : next_state = S_FETCH_0;
                S_CPL_AB_4 : next_state = S_FETCH_0;
                S_SHL_A_4 : next_state = S_FETCH_0;
                S_SHL_B_4 : next_state = S_FETCH_0;
                S_SHR_A_4 : next_state = S_FETCH_0;
                S_SHR_B_4 : next_state = S_FETCH_0;
                S_INC_A_4 : next_state = S_FETCH_0;
                S_INC_B_4 : next_state = S_FETCH_0;
                S_DEC_A_4 : next_state = S_FETCH_0;
                S_DEC_B_4 : next_state = S_FETCH_0;
                //OPERAÇÕES PILHA
                S_PUSH_4 : next_state = S_PUSH_5;
                S_PUSH_5 : next_state = S_PUSH_6;
                S_PUSH_6 : next_state = S_PUSH_7;
                S_PUSH_7 : next_state = S_FETCH_0;
                S_POP_4 : next_state = S_POP_5;
                S_POP_5 : next_state = S_POP_6;
                S_POP_6 : next_state = S_FETCH_0;
                //NOP
                S_NOP_4 : next_state = S_FETCH_0;
                //IN/OUT


                // Next state logic for other states goes here...
                default: next_state = S_FETCH_0; // Default case to avoid latches
            endcase
       end
  
	//portas de saida(comandos)
  	always @ (current_state)
        begin: OUTPUT_LOGIC
            case (current_state)
                S_FETCH_0 : begin 
                    //-- Put PC onto MAR to provide address of Opcode
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "000"=PC, "001"=A, "010"=B , 011=C, 100 = T
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_FETCH_1 : begin 
                    //-- Increment PC, Opcode will be available next state
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_FETCH_2 : begin 
                    //-- ir recebe informação da memória
                    IR_Load = 1;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	//***********
              	//LDA_DIR
              	S_LDA_DIR_4 : begin 
                  //-- MAR recebe o PC(nesse caso o operand)
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDA_DIR_5 : begin 
                    //-- passa o PC enquanto o MAR recebe o PC
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDA_DIR_6 : begin 
                    //-- registrador A recebe informação da memória
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDA_DIR_7 : begin 
                    //-- da tempo para receber
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDA_DIR_8 : begin 
                    //-- registrador A recebe informação da memória
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 1;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	//LDB_DIR
              	S_LDB_DIR_4 : begin 
                  //-- MAR recebe o PC(nesse caso o operand)
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDB_DIR_5 : begin 
                    //-- passa o PC enquanto o MAR recebe o PC atual
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDB_DIR_6 : begin 
                  	//-- MAR recebe informação da memória(local onde tá o valor a ser armazenado)
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDB_DIR_7 : begin 
                    //-- so pra da tempo da memoria responder
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDB_DIR_8 : begin 
                    //-- registrador B recebe informação da memória
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 1;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	//LDC_DIR
              	S_LDC_DIR_4 : begin 
                  //-- MAR recebe o PC(nesse caso o operand)
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDC_DIR_5 : begin 
                    //-- passa o PC enquanto o MAR recebe o PC atual
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDC_DIR_6 : begin 
                  	//-- MAR recebe informação da memória(local onde tá o valor a ser armazenado)
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDC_DIR_7 : begin 
                    //-- so pra da tempo da memoria responder
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDC_DIR_8 : begin 
                    //-- registrador c recebe informação da memória
                    IR_Load = 0; 
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	//****************************
              	//LDA_IMM
              	S_LDA_IMM_4 : begin 
                  //-- MAR recebe o PC(nesse caso o operand)
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDA_IMM_5 : begin 
                    //-- passa o PC enquanto o MAR recebe o PC
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDA_IMM_6 : begin 
                    //-- registrador A recebe informação da memória
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 1;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	//LDB_IMM
              	S_LDB_IMM_4 : begin 
                  //-- MAR recebe o PC(nesse caso o operand)
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDB_IMM_5 : begin 
                    //-- passa o PC enquanto o MAR recebe o PC
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDB_IMM_6 : begin 
                    //-- B recebe informação da memória
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 1;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	//LDC_IMM
              	S_LDC_IMM_4 : begin 
                  //-- MAR recebe o PC(nesse caso o operand)
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDC_IMM_5 : begin 
                    //-- passa o PC enquanto o MAR recebe o PC
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_LDC_IMM_6 : begin 
                    //-- C recebe informação da memória
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	//**********************************************
              	//STA_DIR
              	S_STA_DIR_4 : begin 
                  //-- MAR recebe o PC(nesse caso o operand)
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_STA_DIR_5 : begin 
                    //-- passa o PC enquanto o MAR recebe o PC
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_STA_DIR_6 : begin 
                    //-- MAR recebe o oprando de onde tem que guardar na memoria
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_STA_DIR_7 : begin 
                    //-- registrador A passa a informação para a memória
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 1;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	//STB_DIR
              	S_STB_DIR_4 : begin 
                  //-- MAR recebe o PC(nesse caso o operand)
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_STB_DIR_5 : begin 
                    //-- passa o PC enquanto o MAR recebe o PC
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_STB_DIR_6 : begin 
                    //-- MAR recebe o oprando de onde tem que guardar na memoria
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_STB_DIR_7 : begin 
                    //-- registrador B passa a informação para a memória
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 1;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	//STC_DIR
              	S_STC_DIR_4 : begin 
                  //-- MAR recebe o PC(nesse caso o operand)
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_STC_DIR_5 : begin 
                    //-- passa o PC enquanto o MAR recebe o PC
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_STC_DIR_6 : begin 
                    //-- MAR recebe o oprando de onde tem que guardar na memoria
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_STC_DIR_7 : begin 
                    //-- registrador C passa a informação para a memória
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 1;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	//********************
              	//ADD_AB
              	S_ADD_AB_4 : begin 
                  	//-- soma e joga o resultado no C
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000; //n sei qual é o ADD
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                //SUB_AB
              	S_SUB_AB_4 : begin 
                  	//-- subtrai e joga o resultado no C
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00001; // sub
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                 //MUL_AB
              	S_MUL_AB_4 : begin 
                  	//-- multiplica e joga o resultado no C
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00010; // multi
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                //DIV_AB
              	S_DIV_AB_4 : begin 
                  	//-- divide e joga o resultado no C
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00011; // divisão
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                //MOD_AB
              	S_MOD_AB_4 : begin 
                  	//-- divide e joga o mod no C
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00100; // mod
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                //CMP_AB
              	S_CMP_AB_4 : begin 
                  	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00001; // sub
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                //NOT
              	S_NOT_A_4 : begin 
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00101; //not
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_NOT_B_4 : begin 
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00101; //not
                    CCR_Load = 1;
                    Bus1_Sel = 3'b010; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_AND_4 : begin 
                  	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00110; // and
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_NAND_4 : begin 
                  	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00111; // nand
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_OR_4 : begin 
                  	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b01000; // or
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_NOR_4 : begin 
                  	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b01001; // nor
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_XOR_4 : begin 
                  	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b01010; // xor
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_XNOR_4 : begin 
                  	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b01011; // xnor
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_CPE_AB_4 : begin 
                  	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b01100; // 
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_CPG_AB_4 : begin 
                  	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b01101; // 
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_CPL_AB_4 : begin 
                  	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b01110; //
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_SHL_A_4 : begin                  	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b01111; //
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_SHL_B_4 : begin                  	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b01111; //
                    CCR_Load = 1;
                    Bus1_Sel = 3'b010; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_SHR_A_4 : begin                   	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b10000; //
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_SHR_B_4 : begin                   	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b10000; //
                    CCR_Load = 1;
                    Bus1_Sel = 3'b010; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_INC_A_4 : begin                   	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b10001; //
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_INC_B_4 : begin                   	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b10001; //
                    CCR_Load = 1;
                    Bus1_Sel = 3'b010; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_DEC_A_4 : begin                   	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b10010; //
                    CCR_Load = 1;
                    Bus1_Sel = 3'b001; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_DEC_B_4 : begin                   	
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 1;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b10010; //
                    CCR_Load = 1;
                    Bus1_Sel = 3'b010; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	//****************************
              	//BRA
              	S_BRA_4 : begin 
                  //-- MAR recebe o PC
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_BRA_5 : begin 
                    //-- demora um ciclo de clock para receber o valor do operand 
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_BRA_6 : begin 
                    //-- atualiza o PC pela posição  dado pelo operand
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 1;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	//************************************************
              	//BEQ
              	S_BEQ_4 : begin 
                  //-- MAR recebe o PC
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_BEQ_5 : begin 
                    //-- demora um ciclo de clock para receber o valor do operand 
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_BEQ_6 : begin 
                    //-- atualiza o PC pela posição  dado pelo operand
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 1;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_BEQ_7 : begin 
                    //-- se a condicional for falsa ele apenas continua normalmnete incrementando o PC
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_BNQ_4 : begin 
                  //-- MAR recebe o PC
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_BNQ_5 : begin 
                    //-- demora um ciclo de clock para receber o valor do operand 
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_BNQ_6 : begin 
                    //-- atualiza o PC pela posição  dado pelo operand
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 1;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_BNQ_7 : begin 
                    //-- se a condicional for falsa ele apenas continua normalmnete incrementando o PC
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_BGT_4 : begin 
                  //-- MAR recebe o PC
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_BGT_5 : begin 
                    //-- demora um ciclo de clock para receber o valor do operand 
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_BGT_6 : begin 
                    //-- atualiza o PC pela posição  dado pelo operand
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 1;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_BGT_7 : begin 
                    //-- se a condicional for falsa ele apenas continua normalmnete incrementando o PC
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_BLT_4 : begin 
                  //-- MAR recebe o PC
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_BLT_5 : begin 
                    //-- demora um ciclo de clock para receber o valor do operand 
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_BLT_6 : begin 
                    //-- atualiza o PC pela posição  dado pelo operand
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 1;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_BLT_7 : begin 
                    //-- se a condicional for falsa ele apenas continua normalmnete incrementando o PC
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                // CALL
                S_CALL_4 : begin 
                  //-- MAR recebe o PC
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 1;
                    Ads_Sel = 0;
                end
              	S_CALL_5 : begin 
                    //-- demora um ciclo de clock para receber o valor do operand 
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_CALL_6 : begin 
                    //-- atualiza o PC pela posição  dado pelo operand
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 1;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_CALL_7 : begin 
                    //-- guardo MAR + 1 na pilha
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b100; //-- "00"=PC, "01"=A, "10"=B, 011 = C, 100 = MAR
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 1;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 1;
                end
                // RET
                S_RET_4 : begin 
                  //-- Escolho como endereçamento o topo
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 1;
                end
              	S_RET_5 : begin 
                    //-- demora um ciclo de clock para receber o valor do operand 
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 1;
                end
              	S_RET_6 : begin 
                    //-- atualiza o PC pela posição  dado pelo operand
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 1;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 1;
                    T_Inc = 0;
                    Ads_Sel = 1;
                end
                S_PUSH_4 : begin 
                  //-- MAR recebe o PC(nesse caso o operand)
                    IR_Load = 0;
                    MAR_Load = 1;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 1;
                    Ads_Sel = 0;
                end
              	S_PUSH_5 : begin 
                    //-- passa o PC enquanto o MAR recebe o PC
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	S_PUSH_6 : begin 
                    //-- registrador TEMP recebe informação da memória
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 1;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                S_PUSH_7 : begin 
                    //-- escrevo TEMP no topo da pilha
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b101; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 1;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 1;
                end
                S_POP_4 : begin 
                  //-- Escolho como endereçamento o topo
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 1;
                end
              	S_POP_5 : begin 
                    //-- demora um ciclo de clock para receber o valor do operand 
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 1;
                end
              	S_POP_6 : begin 
                    //-- desempilha o valor no reg A
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 1;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 1;
                    T_Inc = 0;
                    Ads_Sel = 1;
                end
                S_NOP_4 : begin 
                    //-- 
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
              	// Output logic for other states goes here...
                default: begin 
                    //-- Default case to prevent latches
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    TEMP_Load = 0;
                    ALU_Sel = 5'b00000;
                    CCR_Load = 0;
                    Bus1_Sel = 3'b000;
                    Bus2_Sel = 2'b00;
                    write = 0;
                    T_Dec = 0;
                    T_Inc = 0;
                    Ads_Sel = 0;
                end
                
            endcase
      end


endmodule

    
//CPU de fato    
module cpu (
    input wire clock,          // Sinal de clock
    input wire reset,          // Sinal de reset
    input wire [7:0] from_memory, // Dados vindos da memória
    output wire write,          // Sinal de escrita para memória
    output wire [7:0] to_memory,  // Dados enviados para memória
    output wire [7:0] address     // Endereço da memória
);

    // Sinais internos para conexão entre control_unit e data_path
    wire IR_Load, MAR_Load, PC_Load, PC_Inc, A_Load, B_Load, C_Load, TEMP_Load, CCR_Load, T_Dec, T_Inc;
    wire [3:0] ALU_Sel;
    wire [2:0] Bus1_Sel;
  	wire [1:0]Bus2_Sel;
  	wire Ads_Sel;
    wire [7:0] IR_Out;
    wire [3:0] CCR_Out;

    // Instanciação do caminho de dados
    data_path dp (
        .clock(clock),
        .reset(reset),
        .Bus1_Sel(Bus1_Sel),
        .Bus2_Sel(Bus2_Sel),
        .Ads_Sel(Ads_Sel),
        .ALU_Sel(ALU_Sel),
        .IR_Load(IR_Load),
        .MAR_Load(MAR_Load),
        .PC_Load(PC_Load),
        .PC_Inc(PC_Inc),
        .A_Load(A_Load),
        .B_Load(B_Load),
        .C_Load(C_Load),
        .TEMP_Load(TEMP_Load),
        .CCR_Load(CCR_Load),
        .from_memory(from_memory),
        .to_memory(to_memory),
        .address(address),
        .IR_Out(IR_Out),
        .CCR_Out(CCR_Out),
        .T_Dec(T_Dec),
        .T_Inc(T_Inc)
    );

    // Instanciação da unidade de controle
    control_unit cu (
        .clock(clock),
        .reset(reset),
        .IR(IR_Out),
        .CCR(CCR_Out),
        .write(write),
        .IR_Load(IR_Load),
        .MAR_Load(MAR_Load),
        .PC_Load(PC_Load),
        .PC_Inc(PC_Inc),
        .A_Load(A_Load),
        .B_Load(B_Load),
        .C_Load(C_Load),
        .TEMP_Load(TEMP_Load),
        .ALU_Sel(ALU_Sel),
        .CCR_Load(CCR_Load),
        .Bus1_Sel(Bus1_Sel),
        .Bus2_Sel(Bus2_Sel),
        .Ads_Sel(Ads_Sel),
        .T_Dec(T_Dec),
        .T_Inc(T_Inc)
    );

endmodule


//***************************************************************


module computer (
    input wire clk,            // Clock
    input wire reset,          // Reset
    input wire [7:0] port_in [15:0],
    output wire [7:0] port_out [15:0]
);
    wire [7:0] address;        // Endereço da memória
    wire [7:0] mem_data_out;   // Dados lidos da memória
    wire [7:0] cpu_data_out;   // Dados enviados pela CPU
    wire write_enable;     // Escrevendo dados? (para simplificar, fixo em 0)
    //wire [7:0] port_in [15:0];
    //wire [7:0] port_out [15:0];


    // Instanciação da CPU
    cpu cpu_inst (
        .clock(clk),
        .reset(reset),
        .write(write_enable),
        .address(address),
        .to_memory(cpu_data_out),
        .from_memory(mem_data_out)
    );

    // Instanciação da Memória
    memory memory_inst (
        .clock(clk),
        .write(write_enable),
        .reset(reset),
        .address(address),
        .data_in(cpu_data_out),
        .data_out(mem_data_out),
        .port_in(port_in),
        .port_out(port_out)
    );

    // Saída final do sistema to ficando doido
    //assign data_out = mem_data_out;

endmodule
