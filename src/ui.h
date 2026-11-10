/* Interface no display OLED: aparelhos, slots, modo gravar e avisos.
 *
 * Toda funcao daqui deve ser chamada so pela thread do main. */
#ifndef UI_H
#define UI_H

#include <zephyr/kernel.h>

/* Gravacoes cruas guardadas na RAM (cada uma ate 2 KB). */
#define UI_RAW_SLOTS 16

#include "ir_library.h"
#include "ir_seq.h"

/* Evento de entrada ja traduzido pelo callback do subsistema de entrada. */
enum ui_event_type {
  UI_EV_ROTATE,   /* encoder girado: value = +1 ou -1 por clique do eixo */
  UI_EV_ENC_DOWN, /* botao do encoder apertado */
  UI_EV_ENC_UP,   /* botao do encoder solto */
  UI_EV_BUTTON,   /* BUTTON0..3 apertado: value = 0..3 */
};

struct ui_event {
  enum ui_event_type type;
  int32_t value;
  uint32_t time_ms; /* k_uptime_get_32() no momento do evento */
};

/* De onde vem o codigo de um slot. */
enum ui_source {
  UI_SRC_NONE,     /* vazio: precisa gravar */
  UI_SRC_FACTORY,  /* banco de codigos (ir_library.h) */
  UI_SRC_RECORDED, /* gravado na placa (tem prioridade) */
};

/* Fila onde o callback de entrada coloca os eventos; o main tira. */
extern struct k_msgq ui_msgq;

/* Inicializa o display. Devolve 0 ou um erro negativo; sem display a
 * interface segue funcionando, so nao desenha. */
int ui_init(void);

/* Trata um evento tirado da ui_msgq. Devolve o slot (do aparelho atual)
 * que deve ser enviado agora, sempre um slot com codigo, ou -1. */
int ui_handle_event(const struct ui_event *evt);

/* Chamada a cada volta do loop: clique longo enquanto o botao ainda esta
 * apertado (gravar, ou apagar a gravacao no modo gravar) e avisos
 * vencidos. */
void ui_tick(void);

/* true enquanto um slot espera um codigo (modo gravar). */
bool ui_is_recording(void);

/* Salva a sequencia no slot do modo gravar, volta para a lista e mostra
 * "Gravado (n)" por 1,5 s. Devolve o slot gravado (do aparelho atual), ou
 * -1 se nao estava no modo gravar. */
int ui_record_sequence(const struct ir_seq *seq);

/* Salva uma gravacao crua no slot do modo gravar e mostra "Gravado cru".
 * Devolve o slot, ou -1 (fora do modo gravar, ou sem espaco: ate
 * UI_RAW_SLOTS gravacoes cruas no total). */
int ui_record_raw(const struct ir_raw *raw);

/* Mostra "Nao reconhecido" por 2 s; o modo gravar continua. */
void ui_record_failed(void);

/* Uma gravacao foi apagada (segurar o encoder no modo gravar). Devolve
 * true uma vez por apagamento, com o aparelho e o slot. */
bool ui_take_erased(int *dev, int *slot);

/* Aparelho atual e troca (cabecalho selecionado + clique). */
int ui_device(void);
void ui_set_device(int dev);
const char *ui_device_name(int dev);
const char *ui_device_slot_name(int dev, int slot);

/* Restaura uma gravacao lida da flash na partida (nao mexe na tela). */
void ui_restore_slot(int dev, int slot, const struct ir_seq *seq);
void ui_restore_raw(int dev, int slot, const struct ir_raw *raw);

/* Slots do aparelho atual. */
const char *ui_slot_name(int slot);
enum ui_source ui_slot_source(int slot);

/* O que o slot envia: uma sequencia (gravada ou de fabrica) ou tempos
 * prontos. A gravacao vale mais que a fabrica. */
const struct ir_seq *ui_slot_sequence(int slot);

/* Tempos prontos do slot (gravacao crua, ou codigo de fabrica de ar
 * Philco): devolve quantos sao e poe o vetor em *d; 0 se nao tem. */
int ui_slot_raw(int slot, const uint16_t **d);

/* Mostra "Enviando..." para o slot dado; -1 volta a tela anterior. */
void ui_set_sending(int slot);

/* Redesenha a tela se algo mudou desde o ultimo desenho. */
void ui_draw(void);

#endif /* UI_H */
