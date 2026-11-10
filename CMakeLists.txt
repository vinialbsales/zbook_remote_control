cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ir_capture)
target_sources(app PRIVATE src/main.c src/ir_decode.c src/ir_seq.c src/ui.c
  src/slot_store.c)
