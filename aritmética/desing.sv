module half_adder (output Sum, Cout, input A ,B);
  xor (Sum, A, B);
  and (Cout, A, B);
endmodule

module full_adder(output Sum, Carry, input Cin, A, B);
  wire HA1_sum, HA1_cout, HA2_cout;

  half_adder M1(.Cout(HA1_cout), .Sum(HA1_sum), .A(A), .B(B));
  half_adder M2(.Sum(Sum), .Cout(HA2_cout), .A(Cin), .B(HA1_sum));

  or M3 (Carry, HA1_cout, HA2_cout);
endmodule

module adder_subtractor(
    output wire [7:0] Sum,
    output wire Cout,
    input wire [7:0] A, B,
    input wire SUB // 0 = Adição, 1 = Subtração
);
    wire [7:0] B_xor;
    wire C1, C2, C3, C4, C5, C6, C7;

    // Calcula o complemento de dois de B caso SUB esteja ativo
    assign B_xor = B ^ {8{SUB}};

    // Cadeia de somadores completos
    full_adder U1 (.Sum(Sum[0]), .Carry(C1), .A(A[0]), .B(B_xor[0]), .Cin(SUB));
    full_adder U2 (.Sum(Sum[1]), .Carry(C2), .A(A[1]), .B(B_xor[1]), .Cin(C1));
    full_adder U3 (.Sum(Sum[2]), .Carry(C3), .A(A[2]), .B(B_xor[2]), .Cin(C2));
    full_adder U4 (.Sum(Sum[3]), .Carry(C4), .A(A[3]), .B(B_xor[3]), .Cin(C3));
    full_adder U5 (.Sum(Sum[4]), .Carry(C5), .A(A[4]), .B(B_xor[4]), .Cin(C4));
    full_adder U6 (.Sum(Sum[5]), .Carry(C6), .A(A[5]), .B(B_xor[5]), .Cin(C5));
    full_adder U7 (.Sum(Sum[6]), .Carry(C7), .A(A[6]), .B(B_xor[6]), .Cin(C6));
    full_adder U8 (.Sum(Sum[7]), .Carry(Cout), .A(A[7]), .B(B_xor[7]), .Cin(C7));
endmodule


module multiplier(
    input wire [7:0] A,  // Multiplicando
    input wire [7:0] B,  // Multiplicador
    output wire [15:0] P // Produto
);
    wire [7:0] pp0, pp1, pp2, pp3, pp4, pp5, pp6, pp7; // Partial products
    wire [15:0] sum1, sum2, sum3, sum4, sum5, sum6, sum7;

    // Calcula os partial products diretamente
    assign pp0 = A & {8{B[0]}};
    assign pp1 = A & {8{B[1]}};
    assign pp2 = A & {8{B[2]}};
    assign pp3 = A & {8{B[3]}};
    assign pp4 = A & {8{B[4]}};
    assign pp5 = A & {8{B[5]}};
    assign pp6 = A & {8{B[6]}};
    assign pp7 = A & {8{B[7]}};

    // Soma os partial products com deslocamento adequado
    assign sum1 = {pp1, 1'b0} + {1'b0, pp0};
    assign sum2 = {pp2, 2'b0} + sum1;
    assign sum3 = {pp3, 3'b0} + sum2;
    assign sum4 = {pp4, 4'b0} + sum3;
    assign sum5 = {pp5, 5'b0} + sum4;
    assign sum6 = {pp6, 6'b0} + sum5;
    assign sum7 = {pp7, 7'b0} + sum6;

    // Produto final
    assign P = sum7;
endmodule

module divider(
    input wire clk, // Clock para controle
    input wire [7:0] dividend, 
    input wire [7:0] divisor,
  	output reg [7:0] quotient, 
    output reg [7:0] remainder
);
    reg [7:0] temp_dividend;
    reg [7:0] temp_quotient;
    integer i;

    always @(posedge clk) begin
      temp_dividend = dividend;
      temp_quotient = 0;

      if (divisor != 0) begin
        for (i = 7; i >= 0; i = i - 1) begin
          if (temp_dividend >= ({7'b0, divisor} << i)) begin  // Expande divisor antes do shift
            temp_dividend = temp_dividend - ({7'b0, divisor} << i);
            temp_quotient = temp_quotient | (1 << i);
          end
        end

        quotient = temp_quotient;
        remainder = temp_dividend;
      end else begin
        quotient = 8'b0;
        remainder = 8'b0;
      end
    end
endmodule

module div(
    input wire clk,
  input wire [7:0] dividend,
  input wire [7:0] divisor,
  output wire [7:0] quotient
);
  wire [7:0] remainder; // Ignorado neste módulo

    // Instancia o módulo divider_4bit apenas se DIV_EN for ativo
    divider u_divider (
      .clk(clk),
        .dividend(dividend),
        .divisor(divisor),
        .quotient(quotient),
        .remainder(remainder)
    );
endmodule

module mod(
    input wire clk,
  input wire [7:0] dividend,
  input wire [7:0] divisor,
  output wire [7:0] remainder
);
  wire [7:0] quotient; // Ignorado neste módulo

    // Instancia o módulo divider_4bit apenas se DIV_EN for ativo
    divider u_divider (
      .clk(clk),
        .dividend(dividend),
        .divisor(divisor),
        .quotient(quotient),
        .remainder(remainder)
    );
endmodule


module inc(
    input wire [7:0] A,        // Valor a ser incrementado
    output wire [7:0] Result,  // Resultado do incremento
    output wire Cout           // Carry out
);
    // Incrementa A adicionando 1 (SUB = 0 para adição)
    adder_subtractor adder (
        .Sum(Result),
        .Cout(Cout),
        .A(A),
        .B(8'b00000001), // Incremento por 1
        .SUB(1'b0)       // Adição
    );
endmodule

module dec(
    input wire [7:0] A,        // Valor a ser decrementado
    output wire [7:0] Result,  // Resultado do decremento
    output wire Cout           // Borrow out
);
    // Decrementa A subtraindo 1 (SUB = 1 para subtração)
    adder_subtractor subtractor (
        .Sum(Result),
        .Cout(Cout),
        .A(A),
        .B(8'b00000001), // Decremento por 1
        .SUB(1'b1)       // Subtração
    );
endmodule

module controlador(
    input wire clk,
    output reg [7:0] Sum,
    output reg [7:0] Quotient,
    output reg [7:0] Remainder,
    output reg Carry,
    output reg [15:0] Product
);

    // Registradores e variáveis internas
    reg [7:0] operand1;
    reg [7:0] operand2;
    reg [7:0] opcode;
    reg [7:0] memory [0:11]; // Ajustado para refletir o tamanho do arquivo (12 palavras)
    integer mem_index = 0;    // Inicializa o índice
    integer word_count = 12;  // Número de palavras no arquivo

    // Sinais intermediários
    wire [7:0] adder_sum, div_quotient, mod_remainder, inc_result, dec_result;
    wire adder_carry, inc_carry, dec_carry;
    wire [15:0] multiplier_product;

    // Instâncias dos módulos necessários
    adder_subtractor adder_inst (
        .Sum(adder_sum),
        .Cout(adder_carry),
        .A(operand1),
        .B(operand2),
        .SUB(opcode == 8'b00000011) // SUB = 1 para subtração
    );

    multiplier multiplier_inst (
        .A(operand1),
        .B(operand2),
        .P(multiplier_product)
    );

    div div_inst (
        .clk(clk),
        .dividend(operand1),
        .divisor(operand2),
        .quotient(div_quotient)
    );

    mod mod_inst (
        .clk(clk),
        .dividend(operand1),
        .divisor(operand2),
        .remainder(mod_remainder)
    );

    inc inc_inst (
        .A(operand1),
        .Result(inc_result),
        .Cout(inc_carry)
    );

    dec dec_inst (
        .A(operand1),
        .Result(dec_result),
        .Cout(dec_carry)
    );

    // Leitura inicial do arquivo
    initial begin
        $readmemb("tarefa2.bin", memory); // Lê as palavras do arquivo (12 no total)
    end

    // Lógica principal do controlador
    always @(posedge clk) begin
        if (mem_index >= word_count) begin
            $display("Fim da memória alcançado");
            $finish;
        end

        opcode = memory[mem_index]; // Lê o próximo comando
        $display("opcode: %b, posição: %d", opcode, mem_index); // Debug do opcode

        // Inicializa os registradores
        Sum = 8'b0;
        Carry = 1'b0;
        Product = 16'b0;
        Quotient = 8'b0;
        Remainder = 8'b0;

        case (opcode)
            8'b00000010: begin // ADD
                operand1 = memory[mem_index + 1];
                operand2 = memory[mem_index + 2];
                #1;
                Sum = adder_sum;
                Carry = adder_carry;
                $display("ADD - A: %b, B: %b", operand1, operand2); // Debug
                mem_index = mem_index + 3;
            end
            8'b00000011: begin // SUB
                operand1 = memory[mem_index + 1];
                operand2 = memory[mem_index + 2];
                #1;
                Sum = adder_sum;
                Carry = adder_carry;
                $display("SUB - A: %b, B: %b", operand1, operand2); // Debug
                mem_index = mem_index + 3;
            end
            8'b00000100: begin // MUL
                operand1 = memory[mem_index + 1];
                operand2 = memory[mem_index + 2];
                #1;
                Product = multiplier_product;
                $display("MUL - A: %b, B: %b", operand1, operand2); // Debug
                mem_index = mem_index + 3;
            end
            8'b00000101: begin // DIV
              operand1 = memory[mem_index + 1];
              operand2 = memory[mem_index + 2];
              #1;
              if (operand2 == 8'b0) begin
                  $display("Erro: Divisão por zero na posição %d", mem_index);
              end else begin
                  @(posedge clk); // Aguarda próximo ciclo de clock
                  Quotient = div_quotient;
                  $display("DIV - A: %b, B: %b, Resultado: %b", operand1, operand2, div_quotient); // Debug
              end
              mem_index = mem_index + 3;
          end
          8'b00000110: begin // MOD
              operand1 = memory[mem_index + 1];
              operand2 = memory[mem_index + 2];
              #1;
              if (operand2 == 8'b0) begin
                  $display("Erro: Modulo por zero na posição %d", mem_index);
              end else begin
                  @(posedge clk); // Aguarda próximo ciclo de clock
                  Remainder = mod_remainder;
                  $display("MOD - A: %b, B: %b, Resultado: %b", operand1, operand2, mod_remainder); // Debug
              end
              mem_index = mem_index + 3;
          end
          	8'b00000111: begin // INC
                operand1 = memory[mem_index + 1];
                #1;
                Sum = inc_result;
                Carry = inc_carry;
                $display("INC - A: %b", operand1); // Debug
                mem_index = mem_index + 2;
            end
            8'b00001000: begin // DEC
                operand1 = memory[mem_index + 1];
                #1;
                Sum = dec_result;
                Carry = dec_carry;
                $display("DEC - A: %b", operand1); // Debug
                mem_index = mem_index + 2;
            end
            default: begin
                $display("Opcode desconhecido: %b na posição %d", opcode, mem_index);
                mem_index = mem_index + 1;
            end
        endcase
    end
endmodule
