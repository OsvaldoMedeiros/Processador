module tb_comparator;

    reg [7:0] A, B;        // Entradas de 8 bits
    wire AeqB, AmaB, AmeB; // Saídas dos comparadores

    // Instanciação dos módulos de comparação
    comparator_eq uut_eq (.AeqB(AeqB), .A(A), .B(B)); // A == B
    comparator_gt uut_gt (.AmaB(AmaB), .A(A), .B(B)); // A > B
    comparator_lt uut_lt (.AmeB(AmeB), .A(A), .B(B)); // A < B

    initial begin
        $display("Iniciando o Testbench para Comparadores");

        // Teste 1: A == B
        A = 8'b00001101; B = 8'b00001101; #10;
        $display("Teste 1: A=%b, B=%b | AeqB=%b, AmaB=%b, AmeB=%b", A, B, AeqB, AmaB, AmeB);

        // Teste 2: A > B
        A = 8'b00010000; B = 8'b00001111; #10;
        $display("Teste 2: A=%b, B=%b | AeqB=%b, AmaB=%b, AmeB=%b", A, B, AeqB, AmaB, AmeB);

        // Teste 3: A < B
        A = 8'b00001110; B = 8'b00010001; #10;
        $display("Teste 3: A=%b, B=%b | AeqB=%b, AmaB=%b, AmeB=%b", A, B, AeqB, AmaB, AmeB);

        // Teste 4: A > B
        A = 8'b11110000; B = 8'b00001111; #10;
        $display("Teste 4: A=%b, B=%b | AeqB=%b, AmaB=%b, AmeB=%b", A, B, AeqB, AmaB, AmeB);

        // Teste 5: A < B
        A = 8'b00000001; B = 8'b11111111; #10;
        $display("Teste 5: A=%b, B=%b | AeqB=%b, AmaB=%b, AmeB=%b", A, B, AeqB, AmaB, AmeB);

        // Teste 6: A == B
        A = 8'b11111111; B = 8'b11111111; #10;
        $display("Teste 6: A=%b, B=%b | AeqB=%b, AmaB=%b, AmeB=%b", A, B, AeqB, AmaB, AmeB);

        // Teste 7: A > B
        A = 8'b10101010; B = 8'b01010101; #10;
        $display("Teste 7: A=%b, B=%b | AeqB=%b, AmaB=%b, AmeB=%b", A, B, AeqB, AmaB, AmeB);

        // Teste 8: A < B
        A = 8'b01010101; B = 8'b10101010; #10;
        $display("Teste 8: A=%b, B=%b | AeqB=%b, AmaB=%b, AmeB=%b", A, B, AeqB, AmaB, AmeB);

        $finish;
    end

endmodule
