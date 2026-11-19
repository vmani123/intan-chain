# No REQUIRES list on purpose: "main" implicitly requires every component in
# the build, so naming them here could only introduce a typo that breaks the
# build without buying anything.
idf_component_register(
    SRCS
        "app_main.c"
        "pipe.c"
        "source_synth.c"
        "source_qspi.c"
        "sink_udp.c"
        "sink_tcp.c"
        "sink_check.c"
        "echo.c"
        "wifi.c"
    INCLUDE_DIRS "."
)

# Ingress selection. Passed on the command line so CI can build both variants
# from one checkout without editing sdkconfig:
#     idf.py -DBENCH_INGRESS=qspi build
if(NOT DEFINED BENCH_INGRESS)
    set(BENCH_INGRESS "synth")
endif()

if(BENCH_INGRESS STREQUAL "qspi")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE BENCH_INGRESS_QSPI=1)
elseif(BENCH_INGRESS STREQUAL "synth")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE BENCH_INGRESS_SYNTH=1)
else()
    message(FATAL_ERROR "BENCH_INGRESS must be 'synth' or 'qspi', got '${BENCH_INGRESS}'")
endif()

message(STATUS "intan-chain ingress: ${BENCH_INGRESS}")

# Wi-Fi band (ESP32-C5 only; ignored on the S3). Also a command-line variable
# so that switching bands recompiles main/ alone:
#     idf.py -DBENCH_WIFI_BAND=5g build       # 5g | 2g | auto (default)
if(NOT DEFINED BENCH_WIFI_BAND)
    set(BENCH_WIFI_BAND "auto")
endif()
if(BENCH_WIFI_BAND STREQUAL "5g")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE CHAIN_WIFI_BAND_5G=1)
elseif(BENCH_WIFI_BAND STREQUAL "2g")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE CHAIN_WIFI_BAND_2G=1)
elseif(NOT BENCH_WIFI_BAND STREQUAL "auto")
    message(FATAL_ERROR "BENCH_WIFI_BAND must be '5g', '2g' or 'auto', got '${BENCH_WIFI_BAND}'")
endif()
message(STATUS "intan-chain wifi band: ${BENCH_WIFI_BAND}")
