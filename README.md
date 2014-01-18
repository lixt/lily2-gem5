lily2-gem5
==========

This is the emulator for LILY2. LILY2 is a DSP core developed by DSP Group, Institute of Microelectronics, Tsinghua University.

To compile the emulator, do:

    scons build/LILY2/gem5.opt CPU_MODELS=HybridCPU

To run it in SE mode, do:

    build/LILY2/gem5.opt configs/example/se.py -c PROGRAM_PATH

To print debug information, do:

    build/LILY2/gem5.opt --debug-flags=DEBUG_FLAGS configs/example/se.py -c PROGRAM_PATH

If you have any question about the emulator, send email to lixiaotian07@gmail.com.
