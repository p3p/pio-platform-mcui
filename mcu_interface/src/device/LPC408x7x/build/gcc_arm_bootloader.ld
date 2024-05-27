/******************************************************************************
 * @file     gcc_arm.ld
 * @brief    GNU Linker Script for Cortex-M based device
 * @version  V2.1.0
 * @date     04. August 2020
 ******************************************************************************/
/*
 * Copyright (c) 2009-2020 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0K, LENGTH = 16K
  RAM   (rwx) : ORIGIN = 0x10000000, LENGTH = 64K - 0x20 /*  top 0x20 used by ISP */

  USB_RAM(rwx) : ORIGIN = 0x20000000, LENGTH = 16K
  ETH_RAM(rwx) : ORIGIN = 0x20004000, LENGTH = 16K
}

/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __copy_table_start__
 *   __copy_table_end__
 *   __zero_table_start__
 *   __zero_table_end__
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 */
ENTRY(Reset_Handler)

SECTIONS
{
  .vectors :
  {
    __vectors_flash_start = LOADADDR (.vectors);
    __vectors_ram_start = .;
    KEEP(*(.vectors))
    *(.vectors.*)
    . = ALIGN(4);
    __vectors_ram_end = .;
  } > RAM AT > FLASH

  .text :
  {
    *(.text*)

    KEEP(*(.init))
    KEEP(*(.fini))

    /* .ctors */
    *crtbegin.o(.ctors)
    *crtbegin?.o(.ctors)
    *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
    *(SORT(.ctors.*))
    *(.ctors)

    /* .dtors */
    *crtbegin.o(.dtors)
    *crtbegin?.o(.dtors)
    *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
    *(SORT(.dtors.*))
    *(.dtors)

    *(.rodata*)

    KEEP(*(.eh_frame*))
  } > FLASH

  /*
   * SG veneers:
   * All SG veneers are placed in the special output section .gnu.sgstubs. Its start address
   * must be set, either with the command line option �--section-start� or in a linker script,
   * to indicate where to place these veneers in memory.
   */
/*
  .gnu.sgstubs :
  {
    . = ALIGN(32);
  } > FLASH
*/

  .ramcode :
  {
    __ramcode_flash_start = LOADADDR (.ramcode);
    __ramcode_ram_start = .;
    *(.glue_7t) *(glue_7)
    *(.ramcode*)
    . = ALIGN(4);
    __ramcode_ram_end = .;
  } > RAM AT > FLASH

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > FLASH

  __exidx_start = .;
  .ARM.exidx :
  {
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } > FLASH
  __exidx_end = .;

  .copy.table :
  {
    . = ALIGN(4);
    __copy_table_start__ = .;

    LONG (__etext)
    LONG (__data_start__)
    LONG ((__data_end__ - __data_start__) / 4)

    LONG (__vectors_flash_start)
    LONG (__vectors_ram_start)
    LONG ((__vectors_ram_end - __vectors_ram_start) / 4)

    LONG (__ramcode_flash_start)
    LONG (__ramcode_ram_start)
    LONG ((__ramcode_ram_end - __ramcode_ram_start) / 4)

    __copy_table_end__ = .;
  } > FLASH

  .zero.table :
  {
    . = ALIGN(4);
    __zero_table_start__ = .;

    LONG (__bss_start__)
    LONG ((__bss_end__ - __bss_start__) / 4)

    __zero_table_end__ = .;
  } > FLASH

  /**
   * Location counter can end up 2byte aligned with narrow Thumb code but
   * __etext is assumed by startup code to be the LMA of a section in RAM
   * which must be 4byte aligned
   */
  __etext = ALIGN (4);

  .data : AT (__etext)
  {
    __data_start__ = .;
    *(.data)
    *(.data.*)

    . = ALIGN(4);
    /* preinit data */
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP(*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);

    . = ALIGN(4);
    /* init data */
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array))
    PROVIDE_HIDDEN (__init_array_end = .);

    . = ALIGN(4);
    /* finit data */
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array))
    PROVIDE_HIDDEN (__fini_array_end = .);

    KEEP(*(.jcr*))
    . = ALIGN(4);
    /* All data end */
    __data_end__ = .;

  } > RAM

  .bss :
  {
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > RAM AT > RAM

  .heap :
  {
      __end__ = .;
      end = __end__;
      *(.heap*)
      __HeapLimit = .;
  } > RAM

  .stack_dummy :
  {
      *(.stack)
  } > RAM

  __StackTop = ORIGIN(RAM) + LENGTH(RAM);
  __StackLimit = __StackTop - SIZEOF(.stack_dummy);
  PROVIDE(__stack = __StackTop);
  PROVIDE(__heapLimit = __HeapLimit);
  PROVIDE(__stackSize = __StackTop - __HeapLimit);

  /* Check if data + heap + stack exceeds RAM limit */
  ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")

  .AHBSRAM0 (NOLOAD):
  {
    Image$$RW_IRAM2$$Base = . ;
    PROVIDE(__AHB0_block_start = .);
    *(AHBSRAM0)
    Image$$RW_IRAM2$$ZI$$Limit = .;
    PROVIDE(__AHB0_dyn_start = .);
    PROVIDE(__AHB0_end = ORIGIN(USB_RAM) + LENGTH(USB_RAM));
  } > USB_RAM

  .AHBSRAM1 (NOLOAD):
  {
    Image$$RW_IRAM3$$Base = . ;
    PROVIDE(__AHB1_block_start = .);
    *(AHBSRAM1)
    Image$$RW_IRAM3$$ZI$$Limit = .;
    PROVIDE(__AHB1_dyn_start = .);
    PROVIDE(__AHB1_end = ORIGIN(ETH_RAM) + LENGTH(ETH_RAM));
  } > ETH_RAM
}
