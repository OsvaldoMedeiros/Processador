//MEMORIA.V


//PROGRAM MEMORY 
module rom_128x8_sync ( // essa é a program memory, é a memória que armazena as intruções e as informações pertinentes para a realização das instruções(opcode e operand). È uma ROM que podem ser armazenadas 128 palavras de 8 bits de tamnho cada(128x8)
    input wire [7:0] address,  // Endereço da ROM
    input wire clock,          // Clock
    output reg [7:0] data_out  // Dados de saída
);
    reg [7:0] rom [0:127];  // Memória ROM de 128 endereços, 8 bits cada


  always @ (address) // verifica se o endereço fornecido está dentro dos limites da program memory
		begin
          if ( (address >= 0) && (address <= 127) )
				EN = 1’b1;
			else
				EN = 1’b0;
			end

	always @ (posedge clock) // verifica se o endereço é coerente antes de enviar os dados para a saída
		begin
			if (EN)
				data_out = ROM[address];
		end

endmodule

	
//DATA MEMORY
module rw_96x8_sync ( // essa é a data memory, é uma memória normal. Aparentemente, serve mais para ajudar a fazer contas maiores e coisinhas desse tipo de suporte ao CPU 
    input wire [7:0] address,  // Endereço da RAM
    input wire clock,          // Clock
    input wire write,          // Sinal de escrita
    input wire [7:0] data_in,  // Dados de entrada para escrita
    output reg [7:0] data_out  // Dados de saída para leitura
);
  reg[7:0] RW[128:223];  // Memória RAM de 96 endereços, 8 bits cada(é uma continuação do passado, por isso começa do 128)

  always @ (address) // verifica se o endereço fornecido está dentro dos limites da data memory
		begin
          if ( (address >= 128) && (address <= 175) )
				EN = 1’b1;
			else
				EN = 1’b0;
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
  reg[7:0] pilha[176:222];  // Memória RAM de 96 endereços, 8 bits cada(é uma continuação do passado, por isso começa do 176)
  reg[7:0] topo[223:223];; //n sei se é melhor colocar o registrador do topo como parte do vetor da pilha o algo separado.Coloquei como fazendo parte pq eu acho q o pc vai ter que armezenar apontar pro registrador que guarda o topo e do topo ir para o local da pilha

  always @ (address) // verifica se o endereço fornecido está dentro dos limites da data memory
		begin
          if ( (address >= 176) && (address <= 223) )
				EN = 1’b1;
			else
				EN = 1’b0;
		end
    
  always @ (posedge clock) // verifica se o endereço é coerente antes de escrever ou enviar os dados
		begin
			if (write && EN)
              RW[address] = data_in;
			else if (!write && EN)
				data_out = RW[address];
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

    // Instanciação da PROGRAM MEMORY
    rom_128x8_sync rom_inst (
        .address(address),
        .clock(clock),
        .data_out(rom_data_out)
    );

    // Instanciação da RAM
    rw_96x8_sync ram_inst (
        .address(address),
        .clock(clock),
        .write(write),
        .data_in(data_in),
        .data_out(ram_data_out)
    );
  
    // Instanciação da PILHA
    pilha (
        .address(address),
        .clock(clock),
        .write(write),
        .data_in(data_in),
        .data_out(ram_data_out)
    );
    

  //vou ficar faltando com uma informação precisa de como essa parte funciona, dps verifiquem na pag 20 do arquivo ou 162 do livro a explicação. Aparentemente, isso é para pegar a informação do data_in e jogar para a porta de saída selecionada pelo endereço.O !reset é para verificar se porta pode ta funcionando e o write se tá podendo receber as informações do data_in. O <= significa atribuição não bloqueada, pelo oq eu entendi siginifica q todas as portas vão receber o valor ao mesmo tempo, n sendo uma atribuição imediata como no "=", para ser algo mais semelhante com os flip flops na vida real na qual só é atualizado quando passa o ciclo do clock
  always @(posedge clock or posedge reset) begin 
    	//-- port_out_00 (address E0)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_00 <= 8’h00;
				else
                  if ((address == 8’hE0) && (write))
					port_out_00 <= data_in;
		end
		//-- port_out_01 (address E1)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_01 <= 8’h00;
				else
                  if ((address == 8’hE1) && (write))
					port_out_01 <= data_in;
		end
    //-- port_out_01 (address E2)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_01 <= 8’h00;
				else
                  if ((address == 8’hE2) && (write))
					port_out_01 <= data_in;
		end
    //-- port_out_01 (address E3)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_01 <= 8’h00;
				else
                  if ((address == 8’hE3) && (write))
					port_out_01 <= data_in;
		end
    //-- port_out_01 (address E4)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_01 <= 8’h00;
				else
                  if ((address == 8’hE4) && (write))
					port_out_01 <= data_in;
		end
    //-- port_out_01 (address E5)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_01 <= 8’h00;
				else
                  if ((address == 8’hE5) && (write))
					port_out_01 <= data_in;
		end
    //-- port_out_01 (address E6)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_01 <= 8’h00;
				else
                  if ((address == 8’hE6) && (write))
					port_out_01 <= data_in;
		end
    //-- port_out_01 (address E7)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_01 <= 8’h00;
				else
                  if ((address == 8’hE7) && (write))
					port_out_01 <= data_in;
		end
    //-- port_out_01 (address E8)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_01 <= 8’h00;
				else
                  if ((address == 8’hE8) && (write))
					port_out_01 <= data_in;
		end
    //-- port_out_01 (address E9)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_01 <= 8’h00;
				else
                  if ((address == 8’hE9) && (write))
					port_out_01 <= data_in;
		end
    //-- port_out_01 (address EA-10)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_01 <= 8’h00;
				else
                  if ((address == 8’hEA) && (write))
					port_out_01 <= data_in;
		end
    //-- port_out_01 (address EB-11)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_01 <= 8’h00;
				else
                  if ((address == 8’hEB) && (write))
					port_out_01 <= data_in;
		end
    //-- port_out_01 (address EC-12)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_01 <= 8’h00;
				else
                  if ((address == 8’hEC) && (write))
					port_out_01 <= data_in;
		end
    //-- port_out_01 (address ED-13)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_01 <= 8’h00;
				else
                  if ((address == 8’hED) && (write))
					port_out_01 <= data_in;
		end
    //-- port_out_01 (address EE-14)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_01 <= 8’h00;
				else
                  if ((address == 8’hEE) && (write))
					port_out_01 <= data_in;
		end
    //-- port_out_01 (address EF-15)
		always @ (posedge clock or negedge reset)
			begin
				if (!reset)
					port_out_01 <= 8’h00;
				else
                  if ((address == 8’hEF) && (write))
					port_out_01 <= data_in;
		end
		
    
    
    //Multiplexador, verifica o endereço para saber qual parte da memória vai ter informações enviadas para o CPU
    
    always @ (address, rom_data_out, rw_data_out,
				port_in_00, port_in_01, port_in_02, port_in_03,
				port_in_04, port_in_05, port_in_06, port_in_07,
				port_in_08, port_in_09, port_in_10, port_in_11,
				port_in_12, port_in_13, port_in_14, port_in_15)
       begin: MUX1
    	if ((address >= 0) && (address <= 127)) 
          data_out = rom_data_out; // Seleciona saída da ROM
      else if ((address >= 128) && (address <= 223)) 
          data_out = rw_data_out; // Seleciona saída da RAM
         else if (address == 8'hF0) //são as portas de entrada que podem ter as informações enviadas de forma direta para o CPU sem ser guardada pela memória
          data_out = port_in_00;
      else if (address == 8'hF1) 
          data_out = port_in_01;
      else if (address == 8'hF2) 
          data_out = port_in_02;
      else if (address == 8'hF3) 
          data_out = port_in_03;
      else if (address == 8'hF4) 
          data_out = port_in_04;
      else if (address == 8'hF5) 
          data_out = port_in_05;
      else if (address == 8'hF6) 
          data_out = port_in_06;
      else if (address == 8'hF7) 
          data_out = port_in_07;
      else if (address == 8'hF8) 
          data_out = port_in_08;
      else if (address == 8'hF9) 
          data_out = port_in_09;
      else if (address == 8'hFA) 
          data_out = port_in_10;
      else if (address == 8'hFB) 
          data_out = port_in_11;
      else if (address == 8'hFC) 
          data_out = port_in_12;
      else if (address == 8'hFD) 
          data_out = port_in_13;
      else if (address == 8'hFE) 
          data_out = port_in_14;
      else if (address == 8'hFF) 
          data_out = port_in_15;
      else 
          data_out = 8'h00; // Valor padrão (caso não haja correspondência)
	end

//***************************************************************

//CPU.V
    

//ALU    
module alu( // n alterei nada na ula enviada pelo gpt
    input wire [7:0] A,         // Operando A
    input wire [7:0] B,         // Operando B
    input wire [2:0] ALU_Sel,   // Selector da operação
    output reg [7:0] ALU_Out,   // Resultado da ALU
    output reg Zero,            // Flag Zero
    output reg Negative,        // Flag Negativo
    output reg Carry,           // Flag Carry
    output reg Overflow         // Flag Overflow
);
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

        // Atualização das flags
        Zero     = (ALU_Out == 8'b0);         // Flag Zero
        Negative = ALU_Out[7];                // Flag Negativo (MSB = 1)
        Carry    = (A + B > 8'b11111111);     // Carry gerado na soma
        Overflow = ((A[7] & B[7] & ~ALU_Out[7]) | (~A[7] & ~B[7] & ALU_Out[7])); // Overflow
    end
endmodule
   
      
    
//DATA_PATH    
module data_path (
    input wire clock,
    input wire reset,
    input wire Bus1_Sel,
    input wire Bus2_Sel,
    input wire [2:0] ALU_Sel,
    input wire IR_Load,
    input wire MAR_Load,
    input wire PC_Load,
    input wire PC_Inc,
    input wire A_Load,
    input wire B_Load,
  	input wire C_Load,
    input wire CCR_Load,
    input wire [7:0] from_memory,
    output wire [7:0] to_memory,
    output wire [7:0] address
);

    // Registradores
  	reg [7:0] IR, MAR, PC, A, B, C, CCR;
    reg [7:0] BUS1, BUS2, ALU_Result;
  
  	
  //Multiplexadores(pag 23)
  always @ (Bus1_Sel, PC, A, B, C)
		begin: MUX_BUS1
			case (Bus1_Sel)
				2’b00 : Bus1 = PC;
				2’b01 : Bus1 = A;
				2’b10 : Bus1 = B;
              	2’b11 : Bus1 = C;
				default : Bus1 = 8’hXX;
			endcase
		end
	always @ (Bus2_Sel, ALU_Result, Bus1, from_memory)
		begin: MUX_BUS2
			case (Bus2_Sel)
				2’b00 : Bus2 = ALU_Result;
				2’b01 : Bus2 = Bus1;
				2’b10 : Bus2 = from_memory;
				default : Bus1 = 8’hXX;
			endcase
		end
	always @ (Bus1, MAR)
		begin
			to_memory = Bus1;
			address = MAR;
		end
  
  //Resgistradores:
  //pag 23(atribuição de valor do IR)
    always @ (posedge clock or negedge reset)
		begin: INSTRUCTION_REGISTER
			if (!reset)
				IR <= 8’h00;
			else
			if (IR_Load)
				IR <= Bus2;
	end
  //pag 23(do MAR dessa vez)
  	always @ (posedge clock or negedge reset)
		begin: MEMORY_ADDRESS_REGISTER
			if (!reset)
				MAR <= 8’h00;
			else
			if (MAR_Load)
				MAR <= Bus2;
	end
  //pag 24(PC- tem um incremento dessa vez ao invés de só receber informação)
	always @ (posedge clock or negedge reset)
		begin: PROGRAM_COUNTER
			if (!reset)
				PC <= 8’h00;
			else
			if (PC_Load)
				PC <= Bus2;
			else if (PC_Inc)
				PC <= MAR + 1;
	end
  //pag 24(registradores gerais)
  	//registrador a
	always @ (posedge clock or negedge reset)
		begin: A_REGISTER
			if (!reset)
				A <= 8’h00;
			else
			if (A_Load)
				A <= Bus2;
	end         
	//registrador b
    always @ (posedge clock or negedge reset)
		begin: B_REGISTER
			if (!reset)
				B <= 8’h00;
			else
			if (B_Load)
				B <= Bus2;
	end
  	//registrador c
    always @ (posedge clock or negedge reset)
		begin: C_REGISTER
			if (!reset)
				C <= 8’h00;
			else
              if (C_Load)
				C <= Bus2;
	end
  //pag 24(CCR)
	always @ (posedge clock or negedge reset)
		begin: CONDITION_CODE_REGISTER
			if (!reset)
				CCR_Result <= 8’h00;
			else
			if (CCR_Load)
				CCR_Result <= NZVC;
	end
  
  
  // Endereço e saída de dados(??? ideia do gpt n sei se mantenho)
    assign address = MAR;
    assign to_memory = A; // Dado a ser enviado para a memória (exemplo)

    

// Instância do módulo ALU
    alu alu_instance (
        .A(A),
        .B(B),
        .ALU_Sel(ALU_Sel),
        .ALU_Out(ALU_Out),
        .Zero(Zero),
        .Negative(Negative),
        .Carry(Carry),
        .Overflow(Overflow)
    );


endmodule


//UNIDADE DE CONTROLE    
module control_unit (
    input wire clock,
    input wire reset,
    input wire write,
    input wire [7:0] from_memory,
    output reg IR_Load,
    output reg MAR_Load,
    output reg PC_Load,
    output reg PC_Inc,
    output reg A_Load,
    output reg B_Load,
  	output reg C_Load,
    output reg CCR_Load,
    output reg [2:0] ALU_Sel,
    output reg Bus1_Sel,
    output reg Bus2_Sel
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
          S_BRA_4 = 28, //-- Branch Always States
          S_BRA_5 = 29,
          S_BRA_6 = 30,
          S_BEQ_4 = 31, //-- Branch if Equal States
          S_BEQ_5 = 32,
          S_BEQ_6 = 33,
          S_BEQ_7 = 34,
          S_ADD_AB_4 = 35; //-- Addition States

	//ESTADO DE MEMORIA
    always @ (posedge clock or negedge reset)
        begin: STATE_MEMORY
            if (!reset)
                current_state <= S_FETCH_0;
            else
                current_state <= next_state;
    end

	//LOGICA DO PROXIMO ESTADO
    always @ (current_state, IR, CCR_Result)
        begin: NEXT_STATE_LOGIC
            case (current_state)
                S_FETCH_0 : next_state = S_FETCH_1; // Path for FETCH instruction
                S_FETCH_1 : next_state = S_FETCH_2;
                S_FETCH_2 : next_state = S_DECODE_3;
                S_DECODE_3 : 
                    if (IR == LDA_IMM) 
                        next_state = S_LDA_IMM_4; // Load A (Immediate)
                    else if (IR == LDA_DIR) 
                        next_state = S_LDA_DIR_4; // Load A (Direct)
                    else if (IR == STA_DIR) 
                        next_state = S_STA_DIR_4; // Store A (Direct)
                    else if (IR == LDB_IMM) 
                        next_state = S_LDB_IMM_4; // Load B (Immediate)
                    else if (IR == LDB_DIR) 
                        next_state = S_LDB_DIR_4; // Load B (Direct)
                    else if (IR == STB_DIR) 
                        next_state = S_STB_DIR_4; // Store B (Direct)
                    else if (IR == BRA) 
                        next_state = S_BRA_4; // Branch Always
                    else if (IR == ADD_AB) 
                        next_state = S_ADD_AB_4; // Add A and B
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
              	//ADD
              	S_ADD_AB_4 : next_state = S_FETCH_0; // Path for ADD_AB instruction
              	//BRA(SEM CONDICIONAL)
              	S_BRA_4 : next_state = S_BRA_5; // Path for BRA instruction
                S_BRA_5 : next_state = S_BRA_6;
                S_BRA_6 : next_state = S_FETCH_0;
              	//BEQ(COM CONDICIONAL)
              	S_BEQ_4 : next_state = S_BEQ_5; // Path for BEQ instruction(caso dê bom)
                S_BEQ_5 : next_state = S_BEQ_6;
                S_BEQ_6 : next_state = S_FETCH_0;
              	S_BEQ_7 : next_state = S_FETCH_0; // Path for BEQ instruction(caso dê ruim)
              
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                end
              	S_LDA_DIR_7 : begin 
                    //-- registrador A recebe informação da memória
                    IR_Load = 0;
                    MAR_Load = ;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b01; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 1;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b01; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 1;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b01; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 1;
                end
              	//********************
              	//ADD_AB
              	S_ADD_AB_4 : begin 
                  	//-- soma e joga o resultado no A
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 0;
                    A_Load = 1;
                    B_Load = 0;
                  	C_Load = 0;
                    ALU_Sel = 3'b000; //n sei qual é o ADD
                    CCR_Load = 1;
                    Bus1_Sel = 2'b01; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
                end
              	S_BRA_6 : begin 
                    //-- atualiza o PC pela posição  dado pelo operand
                    IR_Load = 0;
                    MAR_Load = 0;
                    PC_Load = 0;
                    PC_Inc = 1;
                    A_Load = 0;
                    B_Load = 0;
                  	C_Load = 0;
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b01; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b10; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00; //-- "00"=PC, "01"=A, "10"=B
                    Bus2_Sel = 2'b00; //-- "00"=ALU, "01"=Bus1, "10"=from_memory
                    write = 0;
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
                    ALU_Sel = 3'b000;
                    CCR_Load = 0;
                    Bus1_Sel = 2'b00;
                    Bus2_Sel = 2'b00;
                    write = 0;
                end
            endcase
      end


endmodule

    
//CPU de fato    
module cpu (
    input wire clock,          // Sinal de clock
    input wire reset,          // Sinal de reset
    input wire [7:0] from_memory, // Dados vindos da memória
    input wire write,          // Sinal de escrita para memória
    output wire [7:0] to_memory,  // Dados enviados para memória
    output wire [7:0] address     // Endereço da memória
);

    // Sinais internos para conexão entre control_unit e data_path
    wire IR_Load, MAR_Load, PC_Load, PC_Inc, A_Load, B_Load, CCR_Load;
    wire [2:0] ALU_Sel;
    wire Bus1_Sel, Bus2_Sel;

    // Instanciação do caminho de dados
    data_path dp (
        .clock(clock),
        .reset(reset),
        .Bus1_Sel(Bus1_Sel),
        .Bus2_Sel(Bus2_Sel),
        .ALU_Sel(ALU_Sel),
        .IR_Load(IR_Load),
        .MAR_Load(MAR_Load),
        .PC_Load(PC_Load),
        .PC_Inc(PC_Inc),
        .A_Load(A_Load),
        .B_Load(B_Load),
        .CCR_Load(CCR_Load),
        .from_memory(from_memory),
        .to_memory(to_memory),
        .address(address)
    );

    // Instanciação da unidade de controle
    control_unit cu (
        .clock(clock),
        .reset(reset),
        .write(write),
        .from_memory(from_memory),
        .IR_Load(IR_Load),
        .MAR_Load(MAR_Load),
        .PC_Load(PC_Load),
        .PC_Inc(PC_Inc),
        .A_Load(A_Load),
        .B_Load(B_Load),
        .ALU_Sel(ALU_Sel),
        .CCR_Load(CCR_Load),
        .Bus1_Sel(Bus1_Sel),
        .Bus2_Sel(Bus2_Sel)
    );

endmodule


//***************************************************************


module computer (
    input wire clk,            // Clock
    input wire reset,          // Reset
    input wire [7:0] instr,    // Instrução de entrada
    output wire [7:0] data_out // Saída final de dados
);
    wire [7:0] address;        // Endereço da memória
    wire [7:0] mem_data_out;   // Dados lidos da memória
    wire [7:0] cpu_data_out;   // Dados enviados pela CPU
    wire write_enable = 0;     // Escrevendo dados? (para simplificar, fixo em 0)

    // Instanciação da CPU
    cpu cpu_inst (
        .clk(clk),
        .reset(reset),
        .instr(instr),
        .address(address),
        .data_out(cpu_data_out),
        .data_in(mem_data_out)
    );

    // Instanciação da Memória
    memory memory_inst (
        .clk(clk),
        .write_enable(write_enable),
        .address(address),
        .data_in(cpu_data_out),
        .data_out(mem_data_out)
    );

    // Saída final do sistema
    assign data_out = mem_data_out;

endmodule
