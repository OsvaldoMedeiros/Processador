# Projeto: Modulando um Processador com Verilog 

Neste projeto modulamos um processador, desde o sistema de memória até as operações da ULA.

## Passo a Passo de Uso

### 1. Escrevendo o arquivo.asm
Utilize os Mnemônico em opcodes.txt no arquivo `file_source.asm`. Esse arquivo será usado como entrada para as etapas seguintes.

### 2. Compilando o Código C
Compile o programa em C com o seguinte comando:

```bash
gcc compiler.c -o compiler
```

Após a compilação, execute o programa com:

```bash
./compiler
```

### 3. Compilando o Código Verilog
Compile o código Verilog do nosso processador com o comando abaixo:

```bash
iverilog -Wall -g2012 -o output.vvp testbench.sv computador.sv
```

#### Parâmetros:
- `-Wall`: Ativa todos os avisos de compilação.
- `-g2012`: Utiliza o padrão SystemVerilog 2012.
- `-o output.vvp`: Gera um arquivo de saída chamado `output.vvp`.

### 4. Executando o Código Verilog
Para simular o arquivo compilado, use:

```bash
vvp output.vvp
```

## Requisitos do Sistema
Certifique-se de ter os seguintes programas instalados:
- **GCC**: Para compilar o código em C.
- **Icarus Verilog (iverilog)**: Para compilar o código Verilog.

## Exemplo

### Arquivo `file_source.asm`:
LDA_IMM  
00000010  
LDB_IMM  
00010011  
MUL  

### Explicação
- Carregamento de um valor imediato no registrador A: valor `00000010` binário.  
- Carregamento de um valor imediato no registrador B: valor `00010011` binário.  
- **C** = **A** x **B**.  

### Arquivo `code_source.bin`:
00011110  
00000010  
00100000  
00010011  
00000100  

### Saída:
IR: 00000100  
MAR: 00000101  
PC: 00000101  
A: 00000010  
B: 00010011  
C: 0000000000100110  
TOPO: 10101111  


