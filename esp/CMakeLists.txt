# Top-level ESP-IDF project for the intan-chain node: ESP32-S3 or ESP32-C5.
#
# Build:
#   idf.py set-target esp32s3               # or esp32c5 (main/target.h)
#   idf.py -DBENCH_INGRESS=synth build      # Stage 0: pattern made on the ESP
#   idf.py -DBENCH_INGRESS=qspi  build      # Stage 2: pattern arrives over QSPI
#
# BENCH_INGRESS only chooses which source task app_main starts. Both source
# files are always compiled, so building either variant still type-checks the
# other; that is deliberate, because the QSPI file is the one nobody exercises
# until hardware exists.
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(intan_chain_esp)
