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
