/*
 *  Advanced Linux Sound Architecture
 *  Copyright (c) by Jaroslav Kysela <perex@perex.cz>
 *
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/init.h>
#include <linux/io.h>
#include <sound/core.h>

static int __init alsa_sound_last_init(void)
{
	int idx, ok = 0;

	printk("MQ=1==SD2_DAT0 reg:%x\n", __raw_readl(ioremap(0x020e02fc, 4)));
	printk("MQ=1==SD2_DAT0 conf reg:%x\n", __raw_readl(ioremap(0x020e06e4, 4)));
	printk("MQ=1==SD2_DAT1 reg:%x\n", __raw_readl(ioremap(0x020e0300, 4)));
	printk("MQ=1==SD2_DAT1 conf reg:%x\n", __raw_readl(ioremap(0x020e06e8, 4)));
	printk("MQ=1==SD2_DAT2 reg:%x\n", __raw_readl(ioremap(0x020e0304, 4)));
	printk("MQ=1==SD2_DAT2 conf reg:%x\n", __raw_readl(ioremap(0x020e06ec, 4)));
	printk("MQ=1==SD2_DAT3 reg:%x\n", __raw_readl(ioremap(0x020e0308, 4)));
	printk("MQ=1==SD2_DAT3 conf reg:%x\n", __raw_readl(ioremap(0x020e06f0, 4)));
	
	printk(KERN_INFO "ALSA device list:\n");
	printk("MQ=1==GPIO_0 reg:%x\n", __raw_readl(ioremap(0x020e020c, 4)));
	printk("MQ=1==CCOSR reg2:%x\n", __raw_readl(ioremap(0x020c4060, 4)));
	printk("MQ=1==GPIO_0 config reg3:%x\n", __raw_readl(ioremap(0x020e05dc, 4)));
	//__raw_writel(0x00, ioremap(0x020e020c, 4));
	__raw_writel(0x10E0180, ioremap(0x020c4060, 4));
	//__raw_writel(0x1f0b0, ioremap(0x020e05dc, 4));
	printk("MQ=2==GPIO_0 reg:%x\n", __raw_readl(ioremap(0x020e020c, 4)));
	printk("MQ=2==CCOSR reg2:%x\n", __raw_readl(ioremap(0x020c4060, 4)));
	printk("MQ=2==GPIO_0 config reg3:%x\n", __raw_readl(ioremap(0x020e05dc, 4)));
	for (idx = 0; idx < SNDRV_CARDS; idx++)
		if (snd_cards[idx] != NULL) {
			printk(KERN_INFO "  #%i: %s\n", idx, snd_cards[idx]->longname);
			ok++;
		}
	if (ok == 0)
		printk(KERN_INFO "  No soundcards found.\n");
	return 0;
}

late_initcall_sync(alsa_sound_last_init);
