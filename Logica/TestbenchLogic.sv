`timescale 1ns / 1ps

module tb_logic_controller;

    
    reg [7:0] opcode, A, B; // entradas do controlador
    wire [7:0] Y;           // saída do controlador (resultado)

    // instanciação do controlador
    logic_controller uut (
        .Y(Y),
        .opcode(opcode),
        .A(A),
        .B(B)
    );

    // começa o testbench
    initial begin
        
        $display("Iniciando o Testbench");
        
        // AND
        opcode = 4'b00000001; A = 4'b00001101; B = 4'b00000110; #10;
        $display("AND: A=%b B=%b => Resultado: Y=%b", A, B, Y);

        // OR
        opcode = 4'b00000010; A = 4'b00001101; B = 4'b00000110; #10;
        $display("OR: A=%b B=%b => Resultado: Y=%b", A, B, Y);

        // XOR
        opcode = 4'b00000011; A = 4'b00001101; B = 4'b00000110; #10;
        $display("XOR: A=%b B=%b => Resultado: Y=%b", A, B, Y);

        // NAND
        opcode = 4'b00000100; A = 4'b00001101; B = 4'b00000110; #10;
        $display("NAND: A=%b B=%b => Resultado: Y=%b", A, B, Y);

        // NOR
        opcode = 4'b00000101; A = 4'b00001101; B = 4'b00000110; #10;
        $display("NOR: A=%b B=%b => Resultado: Y=%b", A, B, Y);

        // XNOR
        opcode = 4'b00000110; A = 4'b00001101; B = 4'b00000110; #10;
        $display("XNOR: A=%b B=%b => Resultado: Y=%b", A, B, Y);

        // NOT
        opcode = 4'b00000111; A = 4'b00001101; B = 4'bxxxxxxxx; #10;  // apenas A é utilizado
        $display("NOT: A=%b => Resultado: Y=%b", A, Y);

        // Finaliza a simulação
        $finish;
    end
endmodule
