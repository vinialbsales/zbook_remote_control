/* Gravacoes na flash, pelo subsistema settings com NVS.
 *
 * Chaves (particao zephyr,settings-partition da placa, storage_partition):
 *   remote/d<D>s<S>  gravacao do slot S do aparelho D (ir_seq_pack, ou
 *                    ir_raw_pack para gravacao crua; o 1o byte diz qual)
 *   remote/dev       aparelho escolhido por ultimo (1 byte)
 *   remote/s<S>      formato antigo (um aparelho so): vai para o aparelho
 *                    "livre" ao carregar
 */
#ifndef SLOT_STORE_H
#define SLOT_STORE_H

#include "ir_seq.h"

struct slot_store_cb {
  int num_devices;
  int max_slots;
  int legacy_device; /* aparelho que recebe as chaves remote/s<S> */
  void (*on_slot)(int dev, int slot, const struct ir_seq *seq);
  void (*on_raw)(int dev, int slot, const struct ir_raw *raw);
  void (*on_device)(int dev);
};

/* Inicia o settings e carrega tudo. Devolve quantas gravacoes foram
 * carregadas, ou um erro negativo (tudo continua so na RAM). */
int slot_store_init(const struct slot_store_cb *cb);

/* Grava a sequencia do slot. Devolve 0 ou um erro negativo. */
int slot_store_save(int dev, int slot, const struct ir_seq *seq);

/* Grava uma gravacao crua (ate ~2 KB). */
int slot_store_save_raw(int dev, int slot, const struct ir_raw *raw);

/* Apaga a gravacao do slot (volta ao codigo de fabrica, se houver). */
int slot_store_erase(int dev, int slot);

/* Lembra o aparelho escolhido. */
int slot_store_save_device(int dev);

#endif /* SLOT_STORE_H */
