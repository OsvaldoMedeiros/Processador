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

//---------------------------------------------------------------
// Somador/Subtrator de 8 bits
//---------------------------------------------------------------
module adder_subtractor(
    output wire [7:0] Sum,
    output wire Cout,
    output wire [3:0] NZVC,   // [N, Z, V, C]
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

    // Flags:
    // N = bit mais significativo do resultado
    // Z = 1 se resultado for zero
    // V = overflow (carry do penúltimo bit XOR carry final)
    // C = carry out final
    assign NZVC[3] = Sum[7];
    assign NZVC[2] = (Sum == 8'b0);
    assign NZVC[1] = C7 ^ Cout; 
    assign NZVC[0] = Cout;
endmodule

//---------------------------------------------------------------
// Multiplicador de 8 bits
//---------------------------------------------------------------
module multiplier(
    input wire [7:0] A,  // Multiplicando
    input wire [7:0] B,  // Multiplicador
    output wire [15:0] P, // Produto
    output wire [3:0] NZVC // [N, Z, V, C]
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

    // Flags:
    // Para 16 bits, consideramos:
    // N = bit 15
    // Z = 1 se produto for todo zero
    // V = 1 se algum bit além do 7 estiver setado (extrapolou 8 bits) - suposição
    // C = não se aplica para multiplicação pura
    assign NZVC[3] = P[15];
    assign NZVC[2] = (P == 16'b0);
    assign NZVC[1] = |P[15:8]; // Overflow se ultrapassar 8 bits
    assign NZVC[0] = 1'b0;
endmodule

//---------------------------------------------------------------
// Divisor de 8 bits
//--------------------------------
module divider(
    input wire clk, // Clock para controle
    input wire [7:0] dividend, 
    input wire [7:0] divisor,
    output reg [7:0] quotient, 
    output reg [7:0] remainder,
    output wire [3:0] NZVC  // [N, Z, V, C]
);
    reg [7:0] temp_dividend;
    reg [7:0] temp_quotient;
    integer i;

    always @(posedge clk) begin
      temp_dividend = dividend;
      temp_quotient = 0;

      if (divisor != 0) begin
        for (i = 7; i >= 0; i = i - 1) begin
          if (temp_dividend >= ({7'b0, divisor} << i)) begin
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

    // Flags:
    // N = bit mais significativo do quociente (outra forma de interpretar)
    // Z = 1 se quociente for zero
    // V = 0 (não estamos detectando overflow de sinal aqui)
    // C = 0 (não se aplica para divisão simples)
    assign NZVC[3] = quotient[7];
    assign NZVC[2] = (quotient == 8'b0);
    assign NZVC[1] = 1'b0;
    assign NZVC[0] = 1'b0;
endmodule

module div(
    input wire clk,
    input wire [7:0] dividend,
    input wire [7:0] divisor,
  output wire [7:0] quotient,
  output wire [3:0] NZVC
);
  wire [7:0] remainder; // Ignorado neste módulo

  divider u_divider (
    .clk(clk),
    .dividend(dividend),
    .divisor(divisor),
    .quotient(quotient),
    .remainder(remainder),
    .NZVC(NZVC)
  );
endmodule

module mod(
    input wire clk,
    input wire [7:0] dividend,
    input wire [7:0] divisor,
  output wire [7:0] remainder,
  output wire [3:0] NZVC
);
  wire [7:0] quotient;    // Ignorado neste módulo

  divider u_divider (
    .clk(clk),
    .dividend(dividend),
    .divisor(divisor),
    .quotient(quotient),
    .remainder(remainder),
    .NZVC(NZVC)
  );
endmodule

//---------------------------------------------------------------
// Incrementador de 8 bits
//---------------------------------------------------------------
module inc(
    input wire [7:0] A,        // Valor a ser incrementado
    output wire [7:0] Result,  // Resultado do incremento
    output wire Cout,          // Carry out
    output wire [3:0] NZVC     // [N, Z, V, C]
);
    // Incrementa A adicionando 1 (SUB = 0 para adição)
    adder_subtractor adder (
        .Sum(Result),
        .Cout(Cout),
      .NZVC(NZVC),
        .A(A),
        .B(8'b00000001),
        .SUB(1'b0)
    );
endmodule

//---------------------------------------------------------------
// Decrementador de 8 bits
//---------------------------------------------------------------
module dec(
    input wire [7:0] A,        // Valor a ser decrementado
    output wire [7:0] Result,  // Resultado do decremento
    output wire Cout,          // Borrow out
    output wire [3:0] NZVC     // [N, Z, V, C]
);
    // Decrementa A subtraindo 1 (SUB = 1)
    adder_subtractor subtractor (
        .Sum(Result),
        .Cout(Cout),
      .NZVC(NZVC),
        .A(A),
        .B(8'b00000001),
        .SUB(1'b1)
    );
endmodule
