module testbench();
  reg clk;
  wire [7:0] Sum, Quotient, Remainder;
  wire [15:0] Product;
  wire Carry;

  // Instanciar o módulo controlador
  controlador uut (
    .clk(clk),
    .Sum(Sum),
    .Carry(Carry),
    .Product(Product),
    .Quotient(Quotient),
    .Remainder(Remainder)
  );

  // Clock
  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  // Monitoramento
  initial begin
    $monitor("Time: %0t | Sum: %0b | Product: %0b | Quotient: %0b | Remainder: %0b | Carry: %0b", 
         $time, Sum, Product, Quotient, Remainder, Carry);

    #500 $finish;
  end
endmodule
