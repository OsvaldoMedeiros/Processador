module tb_computer;

    // Declaração de sinais
    reg clk;                   // Clock
    reg reset;                 // Reset
    wire [7:0] port_in_data [15:0];
    wire [7:0] port_out_data [15:0];

    // Instanciação do módulo `computer`
    computer uut (
        .clk(clk),
        .reset(reset),
      .port_in_data(port_in_data),
      .port_out_data(port_out_data)
    );

    // Geração de clock com período de 10 unidades de tempo
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // Alterna o sinal do clock a cada 5 unidades de tempo
    end

    // Processo de simulação
    initial begin
        // Inicialização
        reset = 1;  // Ativa o reset inicialmente
        #10;        // Aguarda 10 unidades de tempo para estabilização do reset
        reset = 0;  // Desativa o reset

        // Simulação de funcionamento
        while(port_out[15] != 1) begin //quando indicado pela porta de saída, a simulção para
        	#5;
        end

        // Finaliza a simulação
        $finish;
    end

endmodule
