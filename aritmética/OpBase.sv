module half_adder (output Sum, Cout, input A ,B);
  xor (Sum,A,B);
  and (Cout,A,B);
endmodule

module full_adder(output Sum, Carry,input Cin, A, B);
  wire HA1_sum, HA1_cout, HA2_cout;
  
  half_adder M1(.Cout(HA1_cout),.Sum(HA1_sum), .A(A), .B(B));
  half_adder M2(.Sum(Sum), .Cout(HA2_cout), .A(Cin), .B(HA1_sum));
  
  or M3 (Carry, HA1_cout, HA2_cout);
endmodule

module adder_subtractor_4bit (
    output wire [3:0] Sum,
    output wire Cout,
    input wire [3:0] A, B,
    input wire SUB // 0 = Adição, 1 = Subtração
);

    wire C1, C2, C3;
    wire [3:0] B_xor;

    // Aplicando XOR em cada bit de B para implementar complemento de dois
    assign B_xor = B ^ {4{SUB}};

  	// Sub = 1: Complemento de 2
  	// Sub = 0: soma normal
    full_adder U1 (.Sum(Sum[0]), .Carry(C1), .A(A[0]), .B(B_xor[0]), .Cin(SUB));
    full_adder U2 (.Sum(Sum[1]), .Carry(C2), .A(A[1]), .B(B_xor[1]), .Cin(C1));
    full_adder U3 (.Sum(Sum[2]), .Carry(C3), .A(A[2]), .B(B_xor[2]), .Cin(C2));
    full_adder U4 (.Sum(Sum[3]), .Carry(Cout), .A(A[3]), .B(B_xor[3]), .Cin(C3));

endmodule

module multiplier_4bit(
    input [3:0] A,  // Multiplicando
    input [3:0] B,  // Multiplicador
    output [7:0] P  // Produto
);
    wire [3:0] pp0, pp1, pp2, pp3; // Partial products
    wire [7:0] sum1, sum2, sum3;

    assign pp0 = A & {4{B[0]}};
    assign pp1 = A & {4{B[1]}};
    assign pp2 = A & {4{B[2]}};
    assign pp3 = A & {4{B[3]}};

    assign sum1 = {pp1, 1'b0} + {1'b0, pp0};
    assign sum2 = {pp2, 2'b00} + sum1;
    assign sum3 = {pp3, 3'b000} + sum2;

    assign P = sum3;
endmodule

module divider_4bit(
    input wire [3:0] dividend, 
    input wire [3:0] divisor,
  input wire [3:0] DIV_EN,  // Habilitação para controle
    output reg [3:0] quotient, 
    output reg [3:0] remainder
);
    reg [3:0] temp_dividend;
    reg [3:0] temp_quotient;
    integer i;

    always @(*) begin
      if (DIV_EN==4'b1000) begin
            temp_dividend = dividend;
            temp_quotient = 0;
            
            if (divisor != 0) begin
                for (i = 3; i >= 0; i = i - 1) begin
                    if (temp_dividend >= ({1'b0, divisor} << i)) begin  // Expande divisor para 5 bits antes do shift
                        temp_dividend = temp_dividend - ({1'b0, divisor} << i);
                        temp_quotient = temp_quotient | (1 << i);
					end
                end
            end

            quotient = temp_quotient;
            remainder = temp_dividend;
        end else begin
            quotient = 4'b0000;
            remainder = 4'b0000;
        end
    end
endmodule

// controlador temporário para meus testes
module controlador(
    output wire [3:0] Sum, 
    output wire [7:0] Product, 
    output wire Carry,
    output wire [3:0] Quotient, 
    output wire [3:0] Remainder
);
  reg [3:0] mem_rb [0:11]; 
  reg [3:0] A, B, DIV_EN;
  reg SUB;
  integer i;


  adder_subtractor_4bit U4(.Sum(Sum), .Cout(Carry), .A(A), .B(B), .SUB(SUB));

  multiplier_4bit U5(.P(Product), .A(A), .B(B));

  divider_4bit U6(.DIV_EN(DIV_EN), .dividend(A), .divisor(B), .quotient(Quotient), .remainder(Remainder));

  initial begin
    $readmemb("tarefa2.bin", mem_rb, 0, 11); 

    for (i = 0; i <= 11; i = i + 1) begin
      if (i + 2 <= 11) begin 
        DIV_EN = 4'b0000; // Desativa a divisão por padrão
        
        if (mem_rb[i] == 4'b0001) begin // Soma
          A = mem_rb[i+1]; 
          B = mem_rb[i+2];
          SUB = 0;
          #10; // Aguarda propagação
        end
        
        else if (mem_rb[i] == 4'b0010) begin // Subtração
          A = mem_rb[i+1]; 
          B = mem_rb[i+2];
          SUB = 1;
          #10; // Aguarda propagação
        end
        
        else if (mem_rb[i] == 4'b0100) begin // Multiplicação
          A = mem_rb[i+1]; 
          B = mem_rb[i+2];
          #10; // Aguarda propagação
        end
        
        else if (mem_rb[i] == 4'b1000) begin // Divisão
          A = mem_rb[i+1]; 
          B = mem_rb[i+2];
          DIV_EN = mem_rb[i]; // Ativa a divisão apenas nesse caso
          #10; // Aguarda propagação
        end
      end
    end
  end
endmodule


