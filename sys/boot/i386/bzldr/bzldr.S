/*
 * Copyright (c) 2000 John Baldwin
 * Copyright (c) 2017 EMC Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <bootargs.h>

/*
 * Memory locations.
 */
		.set MEM_PAGE_SIZE,0x1000	# memory page size, 4k
		.set MEM_ARG,0x900		# Arguments at start
		.set MEM_ARG_BTX,0xa100		# Where we move them to so the
						#  BTX client can see them
		.set MEM_ARG_SIZE,0x18		# Size of the arguments
		.set MEM_BTX_ADDRESS,0x9000	# where BTX lives
		.set MEM_BTX_ENTRY,0x9010	# where BTX starts to execute
		.set MEM_BTX_OFFSET,MEM_PAGE_SIZE # offset of BTX in the loader
		.set MEM_BTX_CLIENT,0xa000	# where BTX clients live
		.set MEM_BIOS_KEYBOARD,0x496	# BDA byte with keyboard bit
		.set MEM_CS,0x0200		# Segment to move our code to
		.set MEM_ORG,MEM_CS<<4		# Address of code segment
		.set MEM_LOADER,0x100000	# Where the loader is
		.set MEM_SS,0x0100		# Where our stack will be
		.set MEM_SP,0x0f00		# Stack pointer
		.set MEM_BZARGS,(MEM_SS<<4) + MEM_SP # Where we'll put the args
/*
 * a.out header fields
 */
		.set AOUT_TEXT,0x04		# text segment size
		.set AOUT_DATA,0x08		# data segment size
		.set AOUT_BSS,0x0c		# zero'd BSS size
		.set AOUT_SYMBOLS,0x10		# symbol table
		.set AOUT_ENTRY,0x14		# entry point
		.set AOUT_HEADER,MEM_PAGE_SIZE	# size of the a.out header
/*
 * Segment selectors.
 */
		.set SEL_SDATA,0x8		# Supervisor data
		.set SEL_RDATA,0x10		# Real mode data
		.set SEL_SCODE,0x18		# PM-32 code
		.set SEL_SCODE16,0x20		# PM-16 code
/*
 * BTX constants
 */
		.set INT_SYS,0x30		# BTX syscall interrupt
/*
 * Bit in MEM_BIOS_KEYBOARD that is set if an enhanced keyboard is present
 */
		.set KEYBOARD_BIT,0x10

		.text
		.code16
		.org 0x0, 0x0

/*
 * Space before the bzImage header. We move the command line here.
 */
cmdline:	.zero 497

/*
 * The bzImage header itself.
 */
setup_sects:	.byte 2		# How many sectors of boot code we have
root_flags:	.word 0		# Root filesystem flags, unused
syssize:	.word 0		# Size of 32-bit code, unused
swap_dev:	.word 0		# Swap device, unused
ram_size:	.word 0		# RAM size, unused
vid_mode:	.word 0		# video mode, unused
root_dev:	.word 0		# Root device, unused
boot_flag:	.word 0xaa55	# Boot flag (magic number)

_start:		jmp start	# jmp instruction

		.ascii "HdrS"	# Magic signature
		.word 0x0203	# Linux/x86 boot protocol version (2.03)

realmode_swtch:	.word 0, 0	# Boot loader hook, unused
start_sys_seg:	.word 0x1000	# Load-low segment, obsolete
version_ptr:	.word bzldr_version-0x200 # Offset of our version string
type_of_loader:	.byte 0		# Boot loader indentifier
loadflags:	.byte 0x1	# Load flags
setup_move_size:.word 0		# Move to high memory size, unused
code32_start:	.long 0x100000	# Boot loader hook
ramdisk_image:	.long 0		# Ramdisk image address
ramdisk_size:	.long 0		# Ramdisk size
bootsect_kludge:.word 0, 0	# Unused
heap_end_ptr:	.word 0		# Free memory after setup end, unused
pad1:		.word 0		# Extended boot loader info, unused
cmd_line_ptr:	.long 0		# Pointer to command line
initrd_addr_max: .long 0xffffffff # Highest legal ramdisk address

b_esdi:		.long 0
b_edx:		.long 0
b_sssp:		.long 0
b_csip:		.long 0

bzldr_version:	.asciz "bzImage Loader"

		.code16
/*
 * BTX program loader for bzImage protocol.
 */
start:		movw $welcome_msg, %si		# %ds:(%si) -> welcome message
		callw putstr			# display the welcome message

/*
 * The Linux/x86 boot protocol places us at 0x10000 which is right where
 * loader itself is going to end up located. Move ourselves somewhere safer.
 */
		movw $MEM_CS, %ax		# load new code segment
		movw %ax, %es			# %es -> new code segment
		movzx setup_sects, %cx		# load sector count
		inc %cx				# add one for the boot sector
		shl $7, %cx			# convert to longs
		xor %si, %si			# zero %si
		xor %di, %di			# zero %di
		mov %si, %fs			# zero %fs (why?)
		cld				# increment mode?
		rep movsl			# copy code to new segment
		mov %ax, %ds			# %ds -> new code segment
		ljmp $MEM_CS, $next		# long jump to new code segment

next:
/*
 * Copy the command line somewhere that's easier to get to.
 */
		movw $MEM_SS, %ax		# load stack segment
		movw %ax, %ss			# use stack segment
		movw $MEM_SP, %sp		# set up stack pointer

		xor %di, %di			# Zero %di so we're pointing
						# at the start of our segment
		mov (cmd_line_ptr), %eax	# Get the linear address of
						# the command line
		and %eax, %eax
		jz endcmd			# Skip if zero

		mov %ax, %si			# Move low word to %si
		shr $4, %eax			# Convert the rest to segment
		and $0x000f, %si		# Starting offset only
		mov %ax, %gs			# %gs -> command line segment
		mov $496, %cx			# Max number of bytes in cmdline
copycmd:	gs lodsb			# load command line char
		and %al, %al			# check for NULL
		jz endcmd			# done if NULL
		stosb				# store command line char
		jmp copycmd			# next char

/*
 * Print the command line.
 */
endcmd:		movw $cmdstart_msg, %si		# load cmdline preamble msg
		callw putstr			# print it
		movw $cmdline, %si		# load cmdline itself
		callw putstr			# print it
		movw $cmdend_msg, %si		# load cmdline postamble msg
		callw putstr			# print it

/*
 * Print out initrd address and size.
 */
		movw $rdstart_msg, %si		# load ramdisk preamble msg
		callw putstr			# print it
		movl (ramdisk_image), %eax	# load ramdisk address
		callw puthex			# print it
		movw $rdsize_msg, %si		# load ramdisk size preamble msg
		callw putstr			# print it
		movl (ramdisk_size), %eax	# load ramdisk size
		callw puthex			# print it
		movw $rdend_msg, %si		# load ramdisk postamble msg
		callw putstr			# print it

/*
 * Zero out the boot args structure.
 */
		movw $MEM_ARG, %bx		# load boot arguments address
		movw %bx, %di			# put it in %di
		xorl %eax, %eax			# zero %eax
		movw $(MEM_ARG_SIZE/4), %cx	# load size in longs to %cx
		rep stosl			# zero out the boot arguments
		orb $KARGS_FLAGS_BZARGS, BA_BOOTFLAGS(%bx)
						# Flag that we have bzImage args

/*
 * Set up bz args structure and add it in to boot args.
 */
		movl $MEM_BZARGS, %edx		# Load bz arg structure address
		movl %edx, 0x10(%bx)		# Store bz arg address in boot args
		movl %edx, %edi			# Load bz arg address to %edi
		xorw %ax, %ax			# Zero %ax
		movw %ax, %es			# Store in %es

		movl $cmdline, %eax		# Load command line address
		addl $MEM_ORG, %eax		# Add start address
		stosl				# Store in bz args
		movl (ramdisk_image), %eax	# Load ramdisk address
		stosl				# Store in bz args
		movl (ramdisk_size), %eax	# Lod ramdisk size
		stosl				# Store in bzards

		movw $MEM_CS, %ax		# Load our base segment
		movw %ax, %es			# Move it to %es

/*
 * Turn on the A20 address line
 */
		callw seta20			# Turn A20 on

/*
 * Relocate the loader and BTX using a very lazy protected mode
 */
		movw $relocate_msg, %si		# Display the
		callw putstr			#  relocation message

		lgdt gdtdesc			# setup our own gdt
		cli				# turn off interrupts
		movl %cr0, %eax			# Turn on
		orb $0x1, %al			#  protected
		movl %eax, %cr0			#  mode
		ljmp $SEL_SCODE,$(pm_start+MEM_ORG) # long jump to clear the
						#  instruction pre-fetch queue

		.code32
pm_start:	movw $SEL_SDATA, %ax		# Initialize
		movw %ax, %ds			#  %ds and
		movw %ax, %es			#  %es to a flat selector
		movl MEM_LOADER+AOUT_ENTRY, %edi # %edi is the destination
		movl $(MEM_LOADER+AOUT_HEADER), %esi # %esi is
						#  the start of the text
						#  segment
		movl MEM_LOADER+AOUT_TEXT, %ecx # %ecx = length of the text
						#  segment
		rep				# Relocate the
		movsb				#  text segment
		addl $(MEM_PAGE_SIZE - 1), %edi	# pad %edi out to a new page
		andl $~(MEM_PAGE_SIZE - 1), %edi #  for the data segment
		movl MEM_LOADER+AOUT_DATA, %ecx	# size of the data segment
		rep				# Relocate the
		movsb				#  data segment
		movl MEM_LOADER+AOUT_BSS, %ecx	# size of the bss
		xorl %eax, %eax			# zero %eax
		addb $3, %cl			# round %ecx up to
		shrl $2, %ecx			#  a multiple of 4
		rep				# zero the
		stosl				#  bss

		movl MEM_LOADER+AOUT_ENTRY, %edx # %edx -> relocated loader
		movl %edx, %esi			# %esi -> relocated loader
		movl %edx, aout_entry+MEM_ORG	# store entry address for later
		addl $MEM_BTX_OFFSET, %esi	# %esi -> BTX in the loader
		movl $MEM_BTX_ADDRESS, %edi	# %edi -> where BTX needs to go
		movzwl 0xa(%esi), %ecx		# %ecx -> length of BTX
		rep				# Relocate
		movsb				#  BTX
		ljmp $SEL_SCODE16, $(pm_16+MEM_ORG) # Jump to 16-bit PM
		.code16
pm_16:		movw $SEL_RDATA, %ax		# Initialize
		movw %ax, %ds			#  %ds and
		movw %ax, %es			#  %es to a real mode selector
		movl %cr0, %eax			# Turn off
		andb $~0x1, %al			#  protected
		movl %eax, %cr0			#  mode
		ljmp $MEM_CS,$pm_end		# Long jump to clear the
						#  instruction pre-fetch queue
pm_end:		sti				# Turn interrupts back on now
		movw $MEM_CS, %ax
		movw %ax, %ds
		xorw %ax, %ax
		movw %ax, %es
		movw $MEM_BTX_CLIENT, %di
		movw $btx_client, %si
		movw $(btx_client_end - btx_client), %cx
		rep movsb

		movw $MEM_ARG, %si
		movw $MEM_ARG_BTX, %di
		movw $(MEM_ARG_SIZE/4), %cx
		rep movsl

		movl aout_entry, %eax
		stosl

		movw $MEM_CS, %ax
		movw %ax, %ds
		movw %ax, %es
		movw $done_msg, %si
		callw putstr

		xorw %ax, %ax
		movw %ax, %ds
		movw %ax, %es
		ljmp $0, $MEM_BTX_ENTRY

		.code16
putc:		movw $0x7, %bx
		movb $0xe, %ah
		int $0x10
		retw

putstr:		lodsb				# load %al from %ds:(%si)
		testb %al,%al			# stop at null
		jnz putstr0			# if the char != null, output it
		retw				# return when null is hit
putstr0:	callw putc			# call BIOS, print char in %al
		jmp putstr			# keep looping

puthex:		movl %eax, %edx
		xorl %eax, %eax
		movb $'0', %al
		callw putc
		movb $'x', %al
		callw putc
		mov $8, %cx
puthex0:	movl %edx, %eax
		shrl $28, %eax
		cmpb $0xa, %al
		jl puthex1
		addb $0x57, %al
		jmp puthex2
puthex1:	addb $0x30, %al
puthex2:	callw putc
		shll $4, %edx
		sub $1, %cx
		jnz puthex0
		retw

/*
 * Enable A20. Put an upper limit on the amount of time we wait for the
 * keyboard controller to get ready (65K x ISA access time). If
 * we wait more than that amount, the hardware is probably
 * legacy-free and simply doesn't have a keyboard controller.
 * Thus, the A20 line is already enabled.
 */
seta20: 	cli				# Disable interrupts
		xor %cx,%cx			# Clear
seta20.1:	inc %cx				# Increment, overflow?
		jz seta20.3			# Yes
		inb $0x64,%al			# Get status
		testb $0x2,%al			# Busy?
		jnz seta20.1			# Yes
		movb $0xd1,%al			# Command: Write
		outb %al,$0x64			#  output port
seta20.2:	inb $0x64,%al			# Get status
		testb $0x2,%al			# Busy?
		jnz seta20.2			# Yes
		movb $0xdf,%al			# Enable
		outb %al,$0x60			#  A20
seta20.3:	sti				# Enable interrupts
		retw				# To caller

/*
 * BTX client to start btxldr
 */
		.code32
btx_client:	movl $(MEM_ARG_BTX-MEM_BTX_CLIENT+MEM_ARG_SIZE-4), %esi
						# %ds:(%esi) -> end
						#  of boot[12] args
		movl $(MEM_ARG_SIZE/4), %ecx	# Number of words to push
		std				# Go backwards
push_arg:	lodsl				# Read argument
		pushl %eax			# Push it onto the stack
		loop push_arg			# Push all of the arguments
		cld				# In case anyone depends on this
		pushl MEM_ARG_BTX-MEM_BTX_CLIENT+MEM_ARG_SIZE # Entry point of
						#  the loader
		pushl %eax			# Emulate a near call
		movl $0x1, %eax			# 'exec' system call
		int $INT_SYS			# BTX system call
btx_client_end:
		.code16

		.p2align 4
/*
 * Global descriptor table.
 */
gdt:		.word 0x0,0x0,0x0,0x0		# Null entry
		.word 0xffff,0x0,0x9200,0xcf	# SEL_SDATA
		.word 0xffff,0x0,0x9200,0x0	# SEL_RDATA
		.word 0xffff,0x0,0x9a00,0xcf	# SEL_SCODE (32-bit)
		.word 0xffff,0x0,0x9a00,0x8f	# SEL_SCODE16 (16-bit)
gdt.1:
/*
 * Pseudo-descriptors.
 */
gdtdesc:	.word gdt.1-gdt-1		# Limit
		.long gdt + MEM_ORG		# Base

welcome_msg:	.asciz	"bzImage Loader 1.00\r\n\n"
cmdstart_msg:	.asciz	"Command line: \""
cmdend_msg:	.asciz	"\"\r\n"
rdstart_msg:	.asciz	"Ramdisk: "
rdsize_msg:	.asciz	"+["
rdend_msg:	.asciz	"]\r\n"
relocate_msg:	.asciz	"Relocating the loader and the BTX\r\n"
done_msg:	.asciz	"Done for now!\r\n"

aout_entry:	.long	0

end: