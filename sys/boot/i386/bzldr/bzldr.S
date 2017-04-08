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
		.set MEM_CS,0x0200
		.set MEM_ORG,MEM_CS<<4		# Our code entry
		.set MEM_LOADER,0x100000	# Where the loader is
		.set MEM_SS,0x0100
		.set MEM_SP,0x0f00
		.set MEM_BZARGS,(MEM_SS<<4) + MEM_SP
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
setup_sects:	.byte 2
root_flags:	.word 0
syssize:	.word 0
swap_dev:	.word 0
ram_size:	.word 0
vid_mode:	.word 0
root_dev:	.word 0
boot_flag:	.word 0xaa55

_start:		jmp start

		.ascii "HdrS"
		.word 0x0203

realmode_swtch:	.word 0, 0
start_sys_seg:	.word 0x1000
version_ptr:	.word bzldr_version-0x200
type_of_loader:	.byte 0
loadflags:	.byte 0x1
setup_move_size:.word 0
code32_start:	.long 0x100000
ramdisk_image:	.long 0
ramdisk_size:	.long 0
bootsect_kludge:.word 0, 0
heap_end_ptr:	.word 0
pad1:		.word 0
cmd_line_ptr:	.long 0
ramdisk_max:	.long 0xffffffff

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

		movw $MEM_CS, %ax
		movw %ax, %es
		movzx setup_sects, %cx
		inc %cx
		shl $7, %cx
		xor %si, %si
		xor %di, %di
		mov %si, %fs
		cld
		rep movsl
		mov %ax, %ds
		ljmp $MEM_CS, $next

next:
/*
 * Copy the command line somewhere that's easier to get to.
 */
		movw $MEM_SS, %ax		# Stack setup.
		movw %ax, %ss
		movw $MEM_SP, %sp

		xor %di, %di			# Zero %di so we're pointing
						# at the start of our segment
		mov (cmd_line_ptr), %eax	# Get the linear address of
						# the command line
		and %eax, %eax
		jz endcmd			# Skip if zero

		mov %ax, %si			# Move low word to %si
		shr $4, %eax			# Convert the rest to segment
		and $0x000f, %si		# Starting offset only
		mov %ax, %gs
		mov $496, %cx			# Max number of bytes in cmdline
copycmd:	gs lodsb
		and %al, %al
		jz endcmd
		stosb
		jmp copycmd

/*
 * Print the command line.
 */
endcmd:		movw $cmdstart_msg, %si
		callw putstr
		movw $cmdline, %si
		callw putstr
		movw $cmdend_msg, %si
		callw putstr

/*
 * Print out initrd address and size.
 */
		movw $rdstart_msg, %si
		callw putstr
		movl (ramdisk_image), %eax
		callw puthex
		movw $rdsize_msg, %si
		callw putstr
		movl (ramdisk_size), %eax
		callw puthex
		movw $rdend_msg, %si
		callw putstr

		movw $MEM_ARG, %bx
		movw %bx, %di
		xorl %eax, %eax
		movw $(MEM_ARG_SIZE/4), %cx
		rep stosl
		orb $KARGS_FLAGS_BZARGS, BA_BOOTFLAGS(%bx)

		movl $MEM_BZARGS, %edx
		movl %edx, 0x10(%bx)
		movl %edx, %edi
		xorw %ax, %ax
		movw %ax, %es

		movl $cmdline, %eax
		addl $MEM_ORG, %eax
		stosl
		movl (ramdisk_image), %eax
		stosl
		movl (ramdisk_size), %eax
		stosl

		movw $MEM_CS, %ax
		movw %ax, %es

		callw seta20

		movw $relocate_msg, %si
		callw putstr

		lgdt gdtdesc
		cli
		movl %cr0, %eax
		orb $0x1, %al
		movl %eax, %cr0
		ljmpl $SEL_SCODE, $(pm_start + MEM_ORG)

		.code32
pm_start:	movw $SEL_SDATA, %ax
		movw %ax, %ds
		movw %ax, %es
		movl MEM_LOADER + AOUT_ENTRY, %edi
		movl $(MEM_LOADER + AOUT_HEADER), %esi
		movl MEM_LOADER + AOUT_TEXT, %ecx
		rep movsb
		addl $(MEM_PAGE_SIZE - 1), %edi
		andl $~(MEM_PAGE_SIZE - 1), %edi
		movl MEM_LOADER + AOUT_DATA, %ecx
		rep movsb
		movl MEM_LOADER + AOUT_BSS, %ecx
		xorl %eax, %eax
		addb $3, %cl
		shrl $2, %ecx
		rep stosl
		movl MEM_LOADER + AOUT_ENTRY, %edx
		movl %edx, %esi
		movl %edx, aout_entry + MEM_ORG
		addl $MEM_BTX_OFFSET, %esi
		movl $MEM_BTX_ADDRESS, %edi
		movzwl 0xa(%esi), %ecx
		rep movsb
		ljmp $SEL_SCODE16, $(pm_16 + MEM_ORG)

		.code16
pm_16:		movw $SEL_RDATA, %ax
		movw %ax, %ds
		movw %ax, %es
		movl %cr0, %eax
		andb $~0x1, %al
		movl %eax, %cr0
		ljmp $MEM_CS,$pm_end
pm_end:		sti
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