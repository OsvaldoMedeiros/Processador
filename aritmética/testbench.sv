module test();
  wire [3:0] sum, Quotient, Remainder;
  wire [7:0] Product;
  wire c_out;
  controlador CONTROLADOR(.Sum(sum), .Carry(c_out), .Product(Product), .Quotient(Quotient), .Remainder(Remainder));

  initial begin
    monitor;
  end
  
  task monitor;
    integer i;
    $monitor("Tempo:%0t -> Sum:%0b, Carry:%0b", $time, sum, c_out);
    //$strobe("Tempo:%0t -> Sum:%0b, Carry:%0b", $time, sum, c_out);
	$monitor("Tempo:%0t -> Quotient:%0b, Remainder:%0b", $time, Quotient, Remainder);
    for (i = 0; i < 5; i = i + 1) begin
      #10; // Espera para atualização de A, B e SUB
      if (Product !== 8'bxxxxxxxx) begin
        $display("Tempo:%0t -> Product:%0b", $time, Product);
      end
    end
    $finish; 
endtask

endmodule
