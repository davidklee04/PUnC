# PUnC
Introduction

PUnC is a 16-bit stored-program computer where all data and instructions are aligned 16-bit words. Both programs and data reside in the same memory unit. This README file provides an overview of PUnC's hardware modules and architecture.

# Hardware Modules

Memory
* 16-bit memory addresses
* Each address points to a full 16-bit data word
* Implements the first 128 entries (expandable if needed)
* Asynchronous reads, synchronous writes
* Dual read ports (one for external debugging)

Register File
* Eight general-purpose 16-bit registers (0x0 to 0x7)
* Register 7 (0x7) used for subroutine calls and returns
* Asynchronous reads, synchronous writes

Condition Codes
* Three 1-bit condition code registers: N (Negative), Z (Zero), and P (Positive)
* Set by arithmetic and load operations
* Only one of N, Z, and P can be 1 at any given time

Program Counter
* 16-bit register used to address memory
* Increments by 1 after fetching and decoding an instruction
* Can be modified by control flow instructions
* Resetting sets PC to zero

Instruction Register
* 16-bit register storing the currently-executing instruction
* Facilitates setting control signals during execution

Instruction Set
PUnC uses the LC3 instruction set, an educational instruction-set architecture (ISA) created by Professors Yale N. Patt and Sanjay J. Patel. For a full specification of the LC3 architecture, refer to the LC3ISA.pdf file included in the assignment files.

