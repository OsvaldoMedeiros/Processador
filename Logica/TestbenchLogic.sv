timescale 1ns / 1ps

module tb_logic_controller;

    
    reg [3:0] opcode, A, B; // entradas do controlador
    wire [3:0] Y;           // saída do controlador (resultado)

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
        opcode = 4'b0001; A = 4'b1101; B = 4'b0110; #10;
        $display("AND: A=%b B=%b => Resultado: Y=%b", A, B, Y);

        // OR
        opcode = 4'b0010; A = 4'b1101; B = 4'b0110; #10;
        $display("OR: A=%b B=%b => Resultado: Y=%b", A, B, Y);

        // XOR
        opcode = 4'b0011; A = 4'b1101; B = 4'b0110; #10;
        $display("XOR: A=%b B=%b => Resultado: Y=%b", A, B, Y);

        // NAND
        opcode = 4'b0100; A = 4'b1101; B = 4'b0110; #10;
        $display("NAND: A=%b B=%b => Resultado: Y=%b", A, B, Y);

        // Testando a operação NOR
        opcode = 4'b0101; A = 4'b1101; B = 4'b0110; #10;
        $display("NOR: A=%b B=%b => Resultado: Y=%b", A, B, Y);

        // Testando a operação XNOR
        opcode = 4'b0110; A = 4'b1101; B = 4'b0110; #10;
        $display("XNOR: A=%b B=%b => Resultado: Y=%b", A, B, Y);

        // Testando a operação NOT
        opcode = 4'b0111; A = 4'b1101; B = 4'bxxxx; #10;  // Apenas A é utilizado
        $display("NOT: A=%b => Resultado: Y=%b", A, Y);

        // Finaliza a simulação
        $finish;
    end
endmodule
