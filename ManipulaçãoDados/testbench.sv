module tb_AllModules;

    // Parâmetros gerais
    localparam WORD_SIZE = 8;  // Tamanho do registrador

    // Sinais para o módulo BitInvert
    reg [WORD_SIZE-1:0] reg_in;
    reg [3:0] pos;
    wire [WORD_SIZE-1:0] reg_out_invert;

    // Sinais para o módulo BitShiftRotate
    reg shift_enable;
    reg shift_dir;
    reg rotate_enable;
    reg rotate_dir;
    wire [WORD_SIZE-1:0] reg_out_shift;
    wire carry_flag_shift;

    // Sinais para o módulo FlagControl
    reg clk, reset;
    reg zero_flag_in, carry_flag_in, overflow_flag_in;
    wire zero_flag, carry_flag, overflow_flag;

    // Instâncias dos módulos
    BitInvert uut1 (
        .reg_in(reg_in),
        .pos(pos),
        .reg_out(reg_out_invert)
    );

    BitShiftRotate uut2 (
        .reg_in(reg_in),
        .shift_enable(shift_enable),
        .shift_dir(shift_dir),
        .rotate_enable(rotate_enable),
        .rotate_dir(rotate_dir),
        .reg_out(reg_out_shift),
        .carry_flag(carry_flag_shift)
    );

    FlagControl uut3 (
        .clk(clk),
        .reset(reset),
        .zero_flag_in(zero_flag_in),
        .carry_flag_in(carry_flag_in),
        .overflow_flag_in(overflow_flag_in),
        .zero_flag(zero_flag),
        .carry_flag(carry_flag),
        .overflow_flag(overflow_flag)
    );

    // Geração do clock
    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        // --- Teste do módulo BitInvert ---
      $display("======= Teste do módulo BitInvert =======");
        reg_in = 8'b10101010; pos = 3;
        #10;
        $display("Inversão na posição %d: %b -> %b", pos, reg_in, reg_out_invert);

        pos = 7;
        #10;
        $display("Inversão na posição %d: %b -> %b", pos, reg_in, reg_out_invert);

        pos = 0;
        #10;
        $display("Inversão na posição %d: %b -> %b", pos, reg_in, reg_out_invert);

        // --- Teste do módulo BitShiftRotate ---
      $display("======= Teste do módulo BitShiftRotate =======");
        reg_in = 8'b11001100; shift_enable = 1; rotate_enable = 0;

        // Deslocamento à esquerda
        shift_dir = 0;
        #10;
        $display("Deslocamento à esquerda: %b -> %b, Carry: %b", reg_in, reg_out_shift, carry_flag_shift);

        // Deslocamento à direita
        shift_dir = 1;
        #10;
        $display("Deslocamento à direita: %b -> %b, Carry: %b", reg_in, reg_out_shift, carry_flag_shift);

        // Rotação à esquerda
        shift_enable = 0; rotate_enable = 1; rotate_dir = 0;
        #10;
        $display("Rotação à esquerda: %b -> %b, Carry: %b", reg_in, reg_out_shift, carry_flag_shift);

        // Rotação à direita
        rotate_dir = 1;
        #10;
        $display("Rotação à direita: %b -> %b, Carry: %b", reg_in, reg_out_shift, carry_flag_shift);

        $display("======= Teste do módulo FlagControl =======");

        // Teste de reset
        reset = 1;
        #10; // Espera para a borda de subida do clock
        reset = 0;

        // Teste 1: Atualizando as flags
        zero_flag_in = 1; carry_flag_in = 1; overflow_flag_in = 0;
        #10; // Espera pela borda de subida do clock
        $display("Atualização das flags: Zero: %b, Carry: %b, Overflow: %b",
            zero_flag, carry_flag, overflow_flag);

        // Teste 2: Atualizando as flags novamente
        zero_flag_in = 0; carry_flag_in = 0; overflow_flag_in = 1;
        #10; // Espera pela borda de subida do clock
        $display("Atualização das flags: Zero: %b, Carry: %b, Overflow: %b",
            zero_flag, carry_flag, overflow_flag);

        $finish;
    end

endmodule
