/*-
 * Copyright (c) 2009 Marcel Moolenaar
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <stand.h>
#include <sys/param.h>
#include <sys/disk.h>
#include <sys/endian.h>
#include <sys/queue.h>
#include <machine/stdarg.h>

#include "bootstrap.h"
#include "disk.h"

#ifndef MD_IMAGE_SIZE
#define	MD_IMAGE_SIZE	0
#endif

#define	MD_BLOCK_SIZE	512

#if (MD_IMAGE_SIZE != 0 && MD_IMAGE_SIZE % MD_BLOCK_SIZE)
#error Image size must be a multiple of 512.
#endif

struct md_entry {
	void	*addr;
	size_t	size;
	STAILQ_ENTRY(md_entry) entries;
};

STAILQ_HEAD(md_list, md_entry) md_list = STAILQ_HEAD_INITIALIZER(md_list);

int
md_add(void *addr, size_t size)
{
	struct md_entry *disk;

	disk = malloc(sizeof(struct md_entry));
	if (disk == NULL) {
		return (ENOMEM);
	}

	disk->addr = addr;
	disk->size = size;
	STAILQ_INSERT_TAIL(&md_list, disk, entries);

	return (0);
}

#if MD_IMAGE_SIZE > 0
/*
 * Preloaded image gets put here.
 * Applications that patch the object with the image can determine
 * the size looking at the start and end markers (strings),
 * so we want them contiguous.
 */
static struct {
	u_char start[MD_IMAGE_SIZE];
	u_char end[128];
} md_image = {
	.start = "MFS Filesystem goes here",
	.end = "MFS Filesystem had better STOP here",
};
#endif

/* devsw I/F */
static int md_init(void);
static int md_strategy(void *, int, daddr_t, size_t, char *, size_t *);
static int md_open(struct open_file *, ...);
static int md_close(struct open_file *);
static int md_print(int);

struct devsw md_dev = {
	"md",
	DEVT_DISK,
	md_init,
	md_strategy,
	md_open,
	md_close,
	noioctl,
	md_print
};

static int
md_init(void)
{
	struct md_entry *disk;
	int unit, preloaded;

	unit = preloaded = 0;
#if MD_IMAGE_SIZE > 0

	disk = malloc(sizeof(struct md_entry));
	if (disk == NULL) {
		return (ENOMEM);
	}

	disk->addr = md_image.start;
	disk->size = MD_IMAGE_SIZE;

	STAILQ_INSERT_HEAD(&md_list, disk, entries);

	preloaded = 1;
#endif

	unit = 0;
	STAILQ_FOREACH(disk, &md_list, entries) {
		printf("md%d at %p (%d bytes%s)\n", unit, disk->addr,
		    disk->size, preloaded ? ", preloaded" : "");
		preloaded = 0;
	}
	return (0);
}

static int
md_strategy(void *devdata, int rw, daddr_t blk, size_t size,
    char *buf, size_t *rsize)
{
	struct disk_devdesc *dev;
	int unit;
	struct md_entry *disk;
	uint64_t disk_blocks;
	size_t ofs;

	dev = (struct disk_devdesc *)devdata;
	unit = dev->d_unit;
	STAILQ_FOREACH(disk, &md_list, entries) {
		if (unit == 0)
			break;
		unit--;
	}
	if (unit != 0)
		return (ENXIO);

	/* Get disk blocks, this value is either for whole disk or for partition */
	if (disk_ioctl(dev, DIOCGMEDIASIZE, &disk_blocks)) {
		/* DIOCGMEDIASIZE does return bytes. */
		disk_blocks /= MD_BLOCK_SIZE;
	} else {
		/* We should not get here. Just try to survive. */
		disk_blocks = disk->size - dev->d_offset;
	}

	if (blk < 0 || blk >= disk_blocks)
		return (EIO);

	if (size % MD_BLOCK_SIZE)
		return (EIO);

	ofs = (blk + dev->d_offset) * MD_BLOCK_SIZE;
	if ((ofs + size) > disk->size)
		size = disk->size - ofs;

	if (rsize != NULL)
		*rsize = size;

	switch (rw & (F_READ | F_WRITE)) {
	case F_READ:
		bcopy(disk->addr + ofs, buf, size);
		return (0);
	case F_WRITE:
		bcopy(buf, disk->addr + ofs, size);
		return (0);
	}

	return (ENODEV);
}

static int
md_open(struct open_file *f, ...)
{
	va_list ap;
	struct disk_devdesc *dev;
	int unit;
	struct md_entry *disk;

	va_start(ap, f);
	dev = va_arg(ap, struct disk_devdesc *);
	va_end(ap);

	unit = dev->d_unit;
	STAILQ_FOREACH(disk, &md_list, entries) {
		if (unit == 0)
			break;
		unit--;
	}
	if (unit != 0)
		return (ENXIO);

	return (disk_open(dev, disk->size, MD_BLOCK_SIZE));
}

static int
md_close(struct open_file *f)
{
	struct disk_devdesc *dev;
	int unit;
	struct md_entry *disk;

	dev = (struct disk_devdesc *)(f->f_devdata);

	disk = STAILQ_FIRST(&md_list);
	if (disk == NULL)
		return (ENXIO);

	if (dev->d_unit > 0) {
		unit = dev->d_unit;

		STAILQ_FOREACH(disk, &md_list, entries) {
			if (unit == 0)
				break;
			unit--;
		}

		if (unit == 0)
			return (ENXIO);
	}

	return (disk_close(dev));
}

static int
md_print(int verbose)
{
	struct md_entry *disk;
	struct disk_devdesc dev;
	int unit, ret;
	static char line[80];

	printf("%s devices:", md_dev.dv_name);
	if (pager_output("\n") != 0)
		return (1);

	unit = 0;

	STAILQ_FOREACH(disk, &md_list, entries) {
		printf("    md%d:     %p (%u bytes):", unit, disk->addr,
		    disk->size);
		if (pager_output("\n") != 0)
			return (1);
		dev.d_dev = &md_dev;
		dev.d_unit = unit;
		dev.d_slice = -1;
		dev.d_partition = -1;
		if (disk_open(&dev, disk->size, MD_BLOCK_SIZE) == 0) {
			snprintf(line, sizeof(line), "    md%d", unit);
			ret = disk_print(&dev, line, verbose);
			disk_close(&dev);
			if (ret != 0)
			    return (ret);
		}
		unit++;
	}

	return (0);
}
