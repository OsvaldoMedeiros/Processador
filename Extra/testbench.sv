module test();
  wire Reset;
  reg Clock_TB;
  initial begin
    Clock_TB = 1'b0;
    monitor;
  end
  always
    begin
    #5 Clock_TB = ~Clock_TB;
    end
  task monitor; 
    #1 $monitor("Clock_TB:%0b", Clock_TB);
  endtask
endmodule
