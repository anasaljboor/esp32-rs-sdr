/* ESP32-S3 memory map for ESP-SDR, replacing esp-hal's memory.x.
 *
 * GNU ld searches the current directory before -L paths for INCLUDE, so this
 * file (in the package root, where cargo runs the linker) wins over the copy
 * esp-hal writes to its OUT_DIR. build.rs checks the resulting layout.
 *
 * The RF dump engine writes snapshots to 0x3FCD0000 (64 KiB), and the C
 * firmware keeps 0x3FCB0000..0x3FCE0000 free for its ring banks and because
 * handing a bank to the engine takes it away from the CPU. Nothing is linked
 * there: DRAM (data, bss, main stack) ends at 0x3FCB0000 and the reclaimed
 * bootloader area used for the heap starts at 0x3FCE0000.
 *
 * Values assume esp-hal's defaults: 32 KiB instruction cache, 64 KiB data
 * cache. Change RESERVE_ICACHE/RESERVE_DCACHE if you change those settings.
 */
RESERVE_ICACHE = 0x8000;
RESERVE_DCACHE = 0x10000;
VECTORS_SIZE = 0x400;

RF_DUMP_START = 0x3FCB0000;
RF_DUMP_END   = 0x3FCE0000;

MEMORY
{
  vectors_seg ( RX )     : ORIGIN = 0x40370000 + RESERVE_ICACHE, len = VECTORS_SIZE
  /* IRAM shares physical SRAM with DRAM; stop at the IRAM alias of RF_DUMP_START. */
  iram_seg ( RX )        : ORIGIN = 0x40370000 + RESERVE_ICACHE + VECTORS_SIZE,
                           len = 0x403A0000 - (0x40370000 + RESERVE_ICACHE + VECTORS_SIZE)

  /* Reclaimed second-stage bootloader memory above the RF dump banks. */
  dram2_seg ( RW )       : ORIGIN = 0x3FCE0000, len = 0x3FCED710 - 0x3FCE0000
  dram_seg ( RW )        : ORIGIN = 0x3FC88000, len = 0x3FCB0000 - 0x3FC88000

  dcache_reclaimed_seg ( RW ) : ORIGIN = 0x3FCF0000, len = 0x3FD00000 - 0x3FCF0000 - RESERVE_DCACHE

  irom_seg ( RX )        : ORIGIN = 0x42000020, len = 32M - 0x20
  drom_seg ( R )         : ORIGIN = 0x3C000020, len = 32M - 0x20

  rtc_fast_seg(RWX)      : ORIGIN = 0x600fe000, len = 8k
  rtc_slow_seg(RW)       : ORIGIN = 0x50000000, len = 8k
}
