`define WORD_SIZE 8

// ----------SetRegist--------
module SetBit (
  input [`WORD_SIZE-1:0] reg_in,
  input [3:0] pos,
  output [`WORD_SIZE-1:0] reg_out
);

  wire [`WORD_SIZE-1:0] mask;
  assign mask = (1 << pos); // Gera máscara com 1 na posição desejada
  assign reg_out = reg_in | mask; // Usa OR para setar o bit em 1

endmodule


//---------- ClearBit---------
module ClearBit (
  input [`WORD_SIZE-1:0] reg_in,
  input [3:0] pos,
  output [`WORD_SIZE-1:0] reg_out
);
  
  wire [`WORD_SIZE-1:0] mask;
  assign mask = ~(1 << pos);
  assign reg_out = reg_in & mask;
  
endmodule


//-------- Deslocamento e rotação de bits----------
module BitShiftRotate (
  input [7:0] reg_in, // registrador de entrada
  input shift_enable, // habilita o deslocamento
  input shift_dir, // 0 esquerda 1 direita
  input rotate_enable, // habilita a rotação
  input rotate_dir, // 0 esquerda 1 direita
  output reg [7:0] reg_out, // registrador de entrada
  output reg carry_flag // flag de carry (bit perdido)
);
  
  always @(*) begin
    if (shift_enable) begin
      if (shift_dir == 0) begin
        reg_out = reg_in << 1; 
        carry_flag = reg_in[`WORD_SIZE-1]; // bit mais significativo
      end else begin
        reg_out = reg_in >> 1;
        carry_flag = reg_in[0]; // bit menos significativo
      end
      
    end else if (rotate_enable) begin
      if (rotate_dir == 0) begin
        reg_out = {reg_in[`WORD_SIZE-2:0], reg_in[`WORD_SIZE-1]}; // {..., ...} concatena as duas partes
        carry_flag = reg_in[`WORD_SIZE-1]; // bit mais significativo vai para carry
      end else begin
        reg_out = {reg_in[0], reg_in[`WORD_SIZE-1:1]};
        carry_flag = reg_in[0]; // bit menos significativo vai para carry
      end
      
    end else begin
      reg_out = reg_in;
      carry_flag = 0;      
    end
  end
endmodule



//-------Controle de flags----------
module FlagControl (
  input clk, reset, 
  input zero_flag_in, //zero flag gerada pela operação
  input carry_flag_in, // carry flag gerada pela operação
  input overflow_flag_in, // overflow flag gerada pela operação
  output reg zero_flag, // zero flag do sistema
  output reg carry_flag, // ------
  output reg overflow_flag // -----
);
  
  
  always @(reset) begin 
    if (reset) begin
      zero_flag <= 0;
      carry_flag <= 0;
      overflow_flag <= 0;
    end else begin
      // flags atualizadas com os valores de entrada
      zero_flag <= zero_flag_in;
      carry_flag <= carry_flag_in;
      overflow_flag <= overflow_flag_in;
    end
  end
endmodule