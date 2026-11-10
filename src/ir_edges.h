/* Filtro de bordas do receptor, chamado pela interrupcao do GPIO47.
 *
 * O que o log da placa mostrou (06/11/2026): com alguns controles (o do
 * projetor Epson), o pino nao recebe so o envelope do sinal, mas a propria
 * portadora de ~38-40 kHz: uma borda a cada ~13 us. Isso causava:
 *
 *   - "sem cabecalho" e "Limite MAX_EDGES: SIM": o pulso de 9 ms do
 *     cabecalho virava ~350 pulsinhos de ~13 us, e as 200 bordas do buffer
 *     acabavam em ~2,5 ms;
 *   - "tempos incoerentes": com bordas tao rapidas, a interrupcao as vezes
 *     perde uma, a contagem pulso/espaco inverte e todo o resto do quadro
 *     sai trocado (pulsos viram espacos).
 *
 * O filtro faz a demodulacao em software:
 *
 *   1. Le o nivel do pino em cada borda, em vez de supor que as bordas
 *      alternam. Se o nivel nao bate com o esperado, uma borda se perdeu
 *      (um pico mais rapido que a interrupcao): a alternancia pulso/espaco
 *      nunca se perde, e um pulsinho de portadora logo depois da descida
 *      estende o pulso ate ele.
 *   2. Um espaco menor que IR_EDGE_MERGE_US e um intervalo da portadora,
 *      nao um espaco do protocolo: a descida e apagada e o pulso continua.
 *
 * Os espacos do protocolo tem centenas de us. Os curtos que o
 * decodificador ainda aceita (bit 1 com esticamento perto do maximo de
 * 1650 us) tem pelo menos ~40 us; os intervalos de portadora medidos
 * ficaram entre 6 e 28 us.
 *
 * C puro e so com inline: compila na placa e nos testes do PC.
 */
#ifndef IR_EDGES_H
#define IR_EDGES_H

#include <stdbool.h>
#include <stdint.h>

#define IR_EDGE_MERGE_US 40

struct ir_edges {
  volatile uint32_t *t; /* instantes: subida, descida, subida... */
  int max;
  volatile int n;
  uint32_t merge;      /* IR_EDGE_MERGE_US no relogio de t */
  uint32_t ignored;    /* bordas que nao bateram com o nivel */
  uint32_t merged;     /* espacos de portadora apagados */
};

/* Trata uma borda: now no mesmo relogio de t, high = nivel lido no pino
 * logo depois da borda. */
static inline void ir_edges_on_edge(struct ir_edges *e, uint32_t now,
                                    bool high) {
  int n = e->n;
  /* Indices pares sao subidas (inicio de pulso), impares, descidas. */
  bool expect_high = (n % 2) == 0;

  if (high != expect_high) {
    if (n == 0) {
      return; /* a captura comeca numa subida */
    }
    /* Uma borda se perdeu: a interrupcao rodou depois de o pino ja ter
     * voltado. No meio de um pulso (esperava descida, pino alto), foi um
     * intervalo de portadora mais curto que a interrupcao: o pulso
     * continua. No meio de um espaco (esperava subida, pino baixo), houve
     * um pulsinho de portadora que ja acabou: se ele veio logo depois da
     * descida registrada, o pulso foi ate agora. */
    if (expect_high && now - e->t[n - 1] < e->merge) {
      e->t[n - 1] = now;
      e->merged++;
    } else {
      e->ignored++;
    }
    return;
  }
  if (expect_high && n > 0 && now - e->t[n - 1] < e->merge) {
    /* Espaco curto demais: era a portadora. O pulso continua. */
    e->n = n - 1;
    e->merged++;
    return;
  }
  if (n < e->max) {
    e->t[n] = now;
    e->n = n + 1;
  }
}

#endif /* IR_EDGES_H */
