// REQUISITOS DO COMPILADOR DE ASSEMBLY(8 bits)
// Identificar e remover comentÃ¡rios;
// Implementar o conjunto mÃ­nimo de instruÃ§Ãµes;
// * Nota mÃ­nima sobe para 8 se:
// Realizar anÃ¡lise semÃ¢ntica (quando identifica uma palavra que nÃ£o pertence ao assembly, acusar erro);
// * Nota mÃ­nima sobe para 9 se:
// Realizar anÃ¡lise sintÃ¡tica (e.g.: se uma operaÃ§Ã£o exigir dois operandos e houver apenas uma linha seguinte, acusar erro de sintaxe).

#include<stdio.h>
#include<string.h>

typedef struct{ // Dicionário para guardar o par de valores opcode e mnemônico
    char *mnem;
    char *opcode;
} Dictionary;

Dictionary opcode_table[] = { // tabela com todos os mnemônicos e seus respectivos opcodes 
    {"ADD","00000010"},
    {"SUB","00000011"},
    {"MUL","00000100"},
    {"DIV","00000110"},
    {"MOD","00000111"},
    {"INC","00000111"},
    {"DEC","00001000"},
    {"NOT","00001001"},
    {"AND","00001010"},
    {"NAND","00001011"},
    {"OR","00001100"},
    {"NOR","00001101"},
    {"XOR","00001110"},
    {"XNOR","00001111"},
    {"SHL","00010000"},
    {"SHR","00010001"},
    {"CP","00010010"},
    {"IN","00010011"},
    {"OUT","00010100"},
    {"HLT","00010101"},
    {"NOP","00010110"},
    {"INT","00010111"},
    {"LDA_IMM","00011000"},
    {"LDA_DIR","00011001"},
    {"LDB_IMM","00011010"},
    {"LDB_DIR","00011011"},
    {"LDC_IMM","00011100"},
    {"LDC_DIR","00011101"},
    {"STA_DIR","00011110"},
    {"STB_DIR","00011111"},
    {"STC_DIR","00100000"},
    {"PUSH","00100001"},
    {"POP","00100010"},
    {"CALL","00100011"},
    {"RET","00100100"},
    {"BRA","00100101"},
    {"BEQ","00100110"},
    {"BNQ","00100111"},
    {"BLT","00101000"},
    {"BGT","00101001"}
};

char *opcode_translator(const char *mnem){ // função para buscar na tabela o opcode correspondente
    for(int i = 0; i < sizeof(opcode_table)/sizeof(opcode_table[0]); i++){
        if(strcmp(mnem,opcode_table[i].mnem)==0){
            return opcode_table[i].opcode;
        }
    }
    return NULL;
}

void remove_comments(char *line){
    char *comment = strpbrk(line, ";#"); // identifica os comentários, caracterizados por ; ou #
    if(comment != NULL) *comment = '\0'; // caso ache um comentário, substitui pelo fim da string
}

int main(){
    FILE *asm = fopen("file_source.asm", "r");
    FILE *bin = fopen("code_source.bin", "wb");
    char data[10];

    if(asm == NULL) printf("Assembly file not found.");
    if(bin == NULL) printf("Error while creating the binary file.");
    else{
        printf("File successfully opened.\n");
        while(fscanf(asm, "%8s", data)==1){
            remove_comments(data); // remove os comentários
            if(strlen(data)>0){ // se após a remoção de comentários, a linha não estiver vazia
                char *opcode = opcode_translator(data);
                if(opcode != NULL){ // se 
                    fputs(opcode, bin);
                    fputc('\n', bin);
                }else{
                    if(strspn(data, "01")== strlen(data) && strlen(data) == 8){ // verifica se a linha contém apenas 0s e 1s e se formam um número de 8 bits
                        fputs(data, bin);
                        fputc('\n', bin);
                    }
                }
            }
        }
    }

    fclose(asm);
    fclose(bin);
    printf("Assymble file compiled to binary successfuly. \n");

    return 0;
}