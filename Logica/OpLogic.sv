`timescale 1ns/1ps

//AND
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

// controlador temporario
module logic_controller(
  output reg [7:0] Y,       // saida da porta logica
  input wire [7:0] opcode,  // codigo da porta
  input wire [7:0] A, B     // as 2 entradas de 8 bits
);

    wire [7:0] and_result, or_result, xor_result, nand_result, nor_result, xnor_result, not_result;

    // instanciamento das portas
    and_gate u1(.Y(and_result), .A(A), .B(B));
    or_gate u2(.Y(or_result), .A(A), .B(B));
    xor_gate u3(.Y(xor_result), .A(A), .B(B));
    nand_gate u4(.Y(nand_result), .A(A), .B(B));
    nor_gate u5(.Y(nor_result), .A(A), .B(B));
    xnor_gate u6(.Y(xnor_result), .A(A), .B(B));
    not_gate u7(.Y(not_result), .A(A));

    always @(*) begin
        case (opcode)
            8'b00000001: Y = and_result;       // AND
            8'b00000010: Y = or_result;        // OR
            8'b00000011: Y = xor_result;       // XOR
            8'b00000100: Y = nand_result;      // NAND
            8'b00000101: Y = nor_result;       // NOR
            8'b00000110: Y = xnor_result;      // XNOR
            8'b00000111: Y = not_result;       // NOT
            default: Y = 8'b00000000;          // Valor padrão
        endcase
    end
endmodule
