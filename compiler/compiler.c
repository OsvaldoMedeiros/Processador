#include<stdio.h>
#include<string.h>

typedef struct{ // Dicionário para guardar o par de valores opcode e mnemônico
    char *mnem;
    char *opcode;
} Dictionary;

Dictionary opcode_table[] = { // tabela com todos os mnemônicos e seus respectivos opcodes 
    {"ADD", "00000010"},
    {"SUB", "00000011"},
    {"MUL", "00000100"},
    {"DIV", "00000101"},
    {"MOD", "00000110"},
    {"INC_A", "00000111"},
    {"INC_B", "00001000"},
    {"DEC_A", "00001001"},
    {"DEC_B", "00001010"},
    {"NOT_A", "00001011"},
    {"NOT_B", "00001100"},
    {"AND", "00001101"},
    {"NAND", "00001110"},
    {"OR", "00001111"},
    {"NOR", "00010000"},
    {"XOR", "00010001"},
    {"XNOR", "00010010"},
    {"SHL_A", "00010011"},
    {"SHL_B", "00010100"},
    {"SHR_A", "00010101"},
    {"SHR_B", "00010110"},
    {"CPE", "00010111"},
    {"CPG", "00011000"},
    {"CPL", "00011001"},
    {"CMP", "00011010"},
    {"IN", "00011011"},
    {"OUT", "00011100"},
    {"NOP", "00011101"},
    {"LDA_IMM", "00011110"},
    {"LDA_DIR", "00011111"},
    {"LDB_IMM", "00100000"},
    {"LDB_DIR", "00100001"},
    {"LDC_IMM", "00100010"},
    {"LDC_DIR", "00100011"},
    {"STA_DIR", "00100100"},
    {"STB_DIR", "00100101"},
    {"STC_DIR", "00100110"},
    {"PUSH", "00100111"},
    {"POP", "00101000"},
    {"CALL", "00101001"},
    {"RET", "00101010"},
    {"BRA", "00101011"},
    {"BEQ", "00101100"},
    {"BNQ", "00101101"},
    {"BLT", "00101110"},
    {"BGT", "00101111"}
};

char *opcode_translator(const char *mnem){ // função para buscar na tabela o opcode correspondente
    for(int i = 0; i < sizeof(opcode_table)/sizeof(opcode_table[0]); i++){
        if(strcmp(mnem,opcode_table[i].mnem)==0){
            return opcode_table[i].opcode;
        }
    }
    return NULL; // se o mnemônico não for encontrado, retorna null
}

void remove_comments(char *line){
    char *comment = strpbrk(line, ";#"); // identifica os comentários, caracterizados por ; ou #
    if(comment != NULL) *comment = '\0'; // caso ache um comentário, substitui pelo fim da string
}

int main(){
    FILE *asm = fopen("file_source.asm", "r");
    FILE *bin = fopen("code_source.bin", "wb");
    char line[256]; // linha do arquivo .asm

    if(asm == NULL) {
        printf("Assembly file not found.");
        return 1;
    }
    if(bin == NULL) {
        printf("Error while creating the binary file.");
        fclose(asm);
        return 1;
    }
    printf("File successfully opened.\n");
    int first_line = 1; // para rastrear as linhas e impedir espaços ou linhas vazias no final
    while(fgets(line, sizeof(line), asm)){ // lê a linha inteira e armazena
        remove_comments(line); // remove os comentários
        char *div = strtok(line, " \t\n"); // divide a linha em "palavras" ou sequência de caracteres
        while(div){ // enquanto houver "palavras"
            if(strlen(div) == 0){ // ignora os divisores vazios
                div = strtok(NULL, " \t\n");
                continue;
            }
            char *opcode = opcode_translator(div);
            if(opcode != NULL){ // se o opcode existir na tabela, substitui e copia para o .bin
                if(!first_line) fputc('\n', bin);
                fputs(opcode, bin);
                first_line = 0;
            }else if(strspn(div, "01")== strlen(div) && strlen(div) == 8){ // verifica se a "palavra" contém apenas 0s e 1s e se formam um número de 8 bits
                if(!first_line) fputc('\n', bin);
                fputs(div, bin);
                first_line = 0;
            }else{ // caso a "palavra" não seja nem um binário 8 bits nem um mnemônico
                printf("Mnemonic '%s' not found in assembly.\n", div);
            }
            div = strtok(NULL, " \t\n"); // passa pra próxima "palavra"
        }
           
    }

    fclose(asm);
    fclose(bin);
    printf("Assembly file compiled to binary successfuly.\n");

    return 0;
}