/* Banco de codigos de fabrica: aparelhos e seus botoes.
 *
 * Cada aparelho tem uma lista de slots. Um slot pode ter um codigo de
 * fabrica (daqui) e/ou uma gravacao feita na placa; a gravacao tem
 * prioridade. Slot sem nenhum dos dois esta vazio e precisa ser gravado.
 *
 * Fonte dos codigos: Flipper-IRDB (github.com/Lucaslhm/Flipper-IRDB,
 * commit d126fb1), pastas TVs/Samsung (60 arquivos) e Projectors/Epson
 * (24 arquivos). Para cada botao foi usado o codigo mais comum entre os
 * arquivos. O Flipper guarda endereco e comando com o bit menos
 * significativo primeiro; aqui os bytes estao na ordem de chegada (MSB
 * primeiro), como o ir_decode devolve: Samsung32 endereco 0x07, comando
 * 0x02 -> 0xE0E040BF.
 *
 * Conferido com a placa: Samsung Ligar 0xE0E040BF e Vol+ 0xE0E0E01F
 * (log de 06/11/2026) e Epson Ligar 0xC1AA09F6 (DEVLOG_EPSON.md).
 *
 * Epson: o controle manda, a cada toque, 0x81C00FF0 e depois o codigo do
 * botao, 108 ms depois (DEVLOG_EPSON.md). No Flipper-IRDB o 0x81C00FF0
 * aparece no lugar de varios botoes diferentes, o que confirma que ele e
 * um prefixo comum. Aqui cada botao do Epson e enviado assim, igual ao
 * controle original.
 *
 * Dados constantes, so com inline: incluido pelo ui.c e pelos testes.
 */
#ifndef IR_LIBRARY_H
#define IR_LIBRARY_H

#include "ir_philco_data.h"
#include "ir_seq.h"

#define IR_LIB_MAX_SLOTS 12

/* Nomes de slot: no maximo 9 caracteres (cabe "nome   gr" em 12). */
#define IR_LIB_SLOT_NAME_MAX 9
/* Nome do aparelho: no maximo 12 caracteres (a linha do cabecalho). */
#define IR_LIB_DEVICE_NAME_MAX 12

struct ir_lib_slot {
  const char *name;
  struct ir_seq seq; /* seq.count == 0: sem codigo de fabrica */
  /* Codigo de fabrica como tempos prontos (pulso, espaco, ..., pulso),
   * para protocolos longos de ar-condicionado. NULL: nao tem. */
  const uint16_t *raw;
  uint16_t raw_n;
};

struct ir_lib_device {
  const char *name;
  int n_slots;
  struct ir_lib_slot slots[IR_LIB_MAX_SLOTS];
};

#define IR_LIB_SAMSUNG(c)                                                 \
  {.count = 1, .codes = {{IR_PROTO_SAMSUNG32, (c)}}, .gap_us = {0}}

#define IR_LIB_EPSON_PREFIX 0x81C00FF0u
#define IR_LIB_EPSON_GAP_US 108000u
#define IR_LIB_EPSON(c)                                                   \
  {.count = 2,                                                            \
   .codes = {{IR_PROTO_NEC, IR_LIB_EPSON_PREFIX}, {IR_PROTO_NEC, (c)}},   \
   .gap_us = {0, IR_LIB_EPSON_GAP_US}}

#define IR_LIB_COOLIX(c)                                                  \
  {.count = 1, .codes = {{IR_PROTO_COOLIX, (c)}}, .gap_us = {0}}

#define IR_LIB_NONE {.count = 0}

#define IR_LIB_RAW(name, arr)                                             \
  {(name), IR_LIB_NONE, (arr), (uint16_t)(sizeof(arr) / sizeof((arr)[0]))}

enum {
  IR_LIB_TV_SAMSUNG = 0,
  IR_LIB_PROJ_EPSON,
  /* Os indices ficam na flash (remote/d<D>s<S>, remote/dev): aparelhos
   * novos entram sempre no fim. */
  IR_LIB_LIVRE,
  IR_LIB_AR_MIDEA,
  IR_LIB_AR_PHILCO,
  IR_LIB_NUM_DEVICES,
};

static const struct ir_lib_device ir_lib_devices[IR_LIB_NUM_DEVICES] = {
    [IR_LIB_TV_SAMSUNG] =
        {
            .name = "TV Samsung",
            .n_slots = 12,
            .slots =
                {
                    {"Ligar", IR_LIB_SAMSUNG(0xE0E040BF)},   /* 35 de 48 */
                    {"Vol +", IR_LIB_SAMSUNG(0xE0E0E01F)},   /* 44 de 45 */
                    {"Vol -", IR_LIB_SAMSUNG(0xE0E0D02F)},   /* 41 de 43 */
                    {"Canal +", IR_LIB_SAMSUNG(0xE0E048B7)}, /* 35 de 38 */
                    {"Canal -", IR_LIB_SAMSUNG(0xE0E008F7)}, /* 35 de 38 */
                    {"Mudo", IR_LIB_SAMSUNG(0xE0E0F00F)},    /* 46 de 47 */
                    {"Entrada", IR_LIB_SAMSUNG(0xE0E0807F)}, /* 32 de 35 */
                    {"Menu", IR_LIB_SAMSUNG(0xE0E058A7)},    /* 28 de 29 */
                    {"Cima", IR_LIB_SAMSUNG(0xE0E006F9)},    /* 38 de 41 */
                    {"Baixo", IR_LIB_SAMSUNG(0xE0E08679)},   /* 37 de 41 */
                    {"OK", IR_LIB_SAMSUNG(0xE0E016E9)},      /* 19 de 21 */
                    {"Voltar", IR_LIB_SAMSUNG(0xE0E01AE5)},  /* 29 de 29 */
                },
        },
    [IR_LIB_PROJ_EPSON] =
        {
            .name = "Proj. Epson",
            .n_slots = 10,
            .slots =
                {
                    {"Ligar", IR_LIB_EPSON(0xC1AA09F6)},   /* 15 de 15 */
                    {"Entrada", IR_LIB_EPSON(0xC1AA31CE)}, /* Source Search */
                    {"Vol +", IR_LIB_EPSON(0xC1AA19E6)},
                    {"Vol -", IR_LIB_EPSON(0xC1AA9966)},
                    {"Mudo", IR_LIB_EPSON(0xC1AAC936)},    /* A/V Mute */
                    {"Menu", IR_LIB_EPSON(0xC1AA59A6)},
                    {"Cima", IR_LIB_EPSON(0xC1AA0DF2)},
                    {"Baixo", IR_LIB_EPSON(0xC1AA4DB2)},
                    {"OK", IR_LIB_EPSON(0xC1AAA15E)},      /* Enter */
                    {"Voltar", IR_LIB_EPSON(0xC1AA21DE)},  /* Esc */
                },
        },
    /* Ar-condicionado da familia Midea Carrier (Springer Midea, Midea,
     * Carrier, Comfee) com controle do protocolo Coolix (IRremoteESP8266:
     * controles Midea RG51/RG52/RG57K, Beko, Toshiba, Tokio...). Cada
     * comando e o estado inteiro: "Frio 22" liga, poe em frio, 22 graus,
     * ventilador automatico.
     *
     * Estado de 24 bits (ir_Coolix.h): 0xB2, depois ventilador << 5 |
     * sensor 0x1F, depois temperatura << 4 | modo << 2. Frio = 0,
     * ventilador automatico = 0b101 (0xBF), temperatura pela tabela
     * kCoolixTempMap. Conferidos com capturas cruas do Flipper-IRDB
     * (ACs/): Desligar em 11 arquivos de marcas diferentes, Frio 18 a 24 e
     * Ventilar no Timberk e no Ballu.
     *
     * Atencao: alguns Midea usam outro protocolo (Midea de 48 bits,
     * controles RG57A/RG57H/RG66/RG10). Se estes nao funcionarem, grave
     * por cima (vira gravacao crua). */
    [IR_LIB_AR_MIDEA] =
        {
            .name = "Ar Midea",
            .n_slots = 10,
            .slots =
                {
                    {"Desligar", IR_LIB_COOLIX(0xB27BE0)},
                    {"Frio 18", IR_LIB_COOLIX(0xB2BF10)},
                    {"Frio 20", IR_LIB_COOLIX(0xB2BF20)},
                    {"Frio 22", IR_LIB_COOLIX(0xB2BF70)},
                    {"Frio 23", IR_LIB_COOLIX(0xB2BF50)},
                    {"Frio 24", IR_LIB_COOLIX(0xB2BF40)},
                    {"Frio 26", IR_LIB_COOLIX(0xB2BFD0)},
                    {"Auto 25", IR_LIB_COOLIX(0xB21FC8)},
                    {"Ventilar", IR_LIB_COOLIX(0xB2BFE4)},
                    {"Oscilar", IR_LIB_COOLIX(0xB26BE0)},
                },
        },
    /* Ar-condicionado Philco: SmartIR (codes/climate/3000.json, 409
     * codigos aprendidos com Broadlink). Quadro unico: cabecalho
     * 9000/4500, 15 bytes com o bit menos significativo primeiro,
     * ultimo byte = soma dos nibbles dos outros. Gerado por
     * tools/gen_philco.py (ir_philco_data.h), que confere o formato e o
     * checksum dos 409. Ventilador automatico, sem oscilar, salvo
     * "22 oscila". */
    [IR_LIB_AR_PHILCO] =
        {
            .name = "Ar Philco",
            .n_slots = 10,
            .slots =
                {
                    IR_LIB_RAW("Desligar", ir_philco_raw_0),
                    IR_LIB_RAW("Frio 18", ir_philco_raw_1),
                    IR_LIB_RAW("Frio 20", ir_philco_raw_2),
                    IR_LIB_RAW("Frio 22", ir_philco_raw_3),
                    IR_LIB_RAW("Frio 23", ir_philco_raw_4),
                    IR_LIB_RAW("Frio 24", ir_philco_raw_5),
                    IR_LIB_RAW("Frio 26", ir_philco_raw_6),
                    IR_LIB_RAW("22 oscila", ir_philco_raw_7),
                    IR_LIB_RAW("Auto 24", ir_philco_raw_8),
                    IR_LIB_RAW("Quente 24", ir_philco_raw_9),
                },
        },
    /* Sem codigos de fabrica: so gravacoes, para qualquer aparelho (e
     * para os slots gravados pelas versoes anteriores do firmware). */
    [IR_LIB_LIVRE] =
        {
            .name = "Livre",
            .n_slots = 8,
            .slots =
                {
                    {"Ligar", IR_LIB_NONE},   {"Vol +", IR_LIB_NONE},
                    {"Vol -", IR_LIB_NONE},   {"Canal +", IR_LIB_NONE},
                    {"Canal -", IR_LIB_NONE}, {"Mudo", IR_LIB_NONE},
                    {"Entrada", IR_LIB_NONE}, {"Menu", IR_LIB_NONE},
                },
        },
};

#endif /* IR_LIBRARY_H */
