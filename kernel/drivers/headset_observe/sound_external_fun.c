/* 
 *
 * Copyright (C) 2009 Rockchip Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/types.h>
#include <linux/module.h>
#include "sound_external_fun.h"

// for codec register
struct codec_external_fun codec_ex_fun = {
	.codec_get_micbias = NULL,
	.codec_set_micbias = NULL,
	.codec_headset_detect = NULL,
	.codec_mic_switch = NULL,
	
	.codec_set_spk  = NULL,
	.codec_set_hp	= NULL,
	.codec_mic_path = NULL,
	.codec_sound_path = NULL
};

// for headset register
struct headset_external_fun headset_ex_fun = {
	.hs_set_spk = NULL,
	.hs_mic_path = NULL,
	.hs_sound_path = NULL
};

int codec_ex_fun_register(struct codec_external_fun * codec_funs)
{
	struct codec_external_fun * ex_fun;
	
	ex_fun = &codec_ex_fun;
	ex_fun->codec_get_micbias = codec_funs->codec_get_micbias;
	ex_fun->codec_set_micbias = codec_funs->codec_set_micbias;
	ex_fun->codec_headset_detect = codec_funs->codec_headset_detect;

	ex_fun->codec_set_spk = codec_funs->codec_set_spk;
	ex_fun->codec_mic_path = codec_funs->codec_mic_path;
	ex_fun->codec_sound_path = codec_funs->codec_sound_path;

	return 0;
}
EXPORT_SYMBOL_GPL(codec_ex_fun_register);

int headset_ex_fun_register(struct headset_external_fun * hs_funs)
{
	struct headset_external_fun * ex_fun;
	
	ex_fun = &headset_ex_fun;
	ex_fun->hs_set_spk = hs_funs->hs_set_spk;
	ex_fun->hs_mic_path = hs_funs->hs_mic_path;
	ex_fun->hs_sound_path = hs_funs->hs_sound_path;

	return 0;
}
EXPORT_SYMBOL_GPL(headset_ex_fun_register);

struct codec_external_fun * get_codec_ex_fun(void)
{
	return &codec_ex_fun;
}
EXPORT_SYMBOL_GPL(get_codec_ex_fun);

struct headset_external_fun * get_headset_ex_fun(void)
{
	return &headset_ex_fun;
}
EXPORT_SYMBOL_GPL(get_headset_ex_fun);

MODULE_DESCRIPTION("Rockchip sound external function");
MODULE_LICENSE("GPL");



