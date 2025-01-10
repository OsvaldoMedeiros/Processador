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
          if ( (address >= 128) && (address <= 223) )
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

    // Instanciação da ROM
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
		//tem mais que essas duas só, vai até E15
    
    
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
    input wire CCR_Load,
    input wire [7:0] from_memory,
    output wire [7:0] to_memory,
    output wire [7:0] address
);

    // Registradores
    reg [7:0] IR, MAR, PC, A, B, CCR;
    reg [7:0] BUS1, BUS2, ALU_Result;
  
  	
  //Multiplexadores(pag 23)
  	always @ (Bus1_Sel, PC, A, B)
		begin: MUX_BUS1
			case (Bus1_Sel)
				2’b00 : Bus1 = PC;
				2’b01 : Bus1 = A;
				2’b10 : Bus1 = B;
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
				A <¼ 8’h00;
			else
			if (A_Load)
				A <¼ Bus2;
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
    output reg CCR_Load,
    output reg [2:0] ALU_Sel,
    output reg Bus1_Sel,
    output reg Bus2_Sel
);

    // Estados da FSM
    typedef enum reg [2:0] {
        FETCH, DECODE, EXECUTE, WRITEBACK
    } state_t;
    state_t current_state, next_state;

    // Transição de estados
    always @(posedge clock or posedge reset) begin
        if (reset) current_state <= FETCH;
        else current_state <= next_state;
    end

    // Lógica da FSM
    always @(*) begin
        // Sinais padrão (desativados)
        IR_Load = 0;
        MAR_Load = 0;
        PC_Load = 0;
        PC_Inc = 0;
        A_Load = 0;
        B_Load = 0;
        CCR_Load = 0;
        ALU_Sel = 3'b000;
        Bus1_Sel = 0;
        Bus2_Sel = 0;
        next_state = current_state;

        case (current_state)
            FETCH: begin
                MAR_Load = 1;   // Carregar endereço no MAR
                PC_Inc = 1;     // Incrementar PC
                next_state = DECODE;
            end
            DECODE: begin
                IR_Load = 1;    // Carregar instrução no IR
                next_state = EXECUTE;
            end
            EXECUTE: begin
                case (from_memory[7:4])  // Decodificar opcode
                    4'b0000: begin // Exemplo: Soma
                        A_Load = 1;
                        B_Load = 1;
                        ALU_Sel = 3'b000;
                        CCR_Load = 1;
                        next_state = WRITEBACK;
                    end
                    // Adicione outros casos de opcode aqui...
                    default: next_state = FETCH;
                endcase
            end
            WRITEBACK: begin
                // Exemplo: Escrever na memória
                if (write) begin
                    Bus2_Sel = 1; // Selecionar dados para BUS2
                end
                next_state = FETCH;
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



