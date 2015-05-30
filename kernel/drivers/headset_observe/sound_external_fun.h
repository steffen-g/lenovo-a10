#ifndef SOUND_EXTERNAL_FUN_H
#define SOUND_EXTERNAL_FUN_H

enum mic_path
{
	MIC_ALL_OFF,
	MIC_PATH_MAIN,
	MIC_PATH_HEADSET,
};

enum sound_path
{
	SND_OFF,
	SND_RCV,
	SND_SPK,
	SND_HP,
	SND_HP_NO_MIC,
	SND_BT,
	SND_SPK_HP,
	SND_RING_SPK,
	SND_RING_HP,
	SND_RING_HP_NO_MIC,
	SND_RING_SPK_HP,
};

// for codec register
struct codec_external_fun {
	bool (*codec_get_micbias)(void);
	bool (*codec_set_micbias)(void);
	int  (*codec_headset_detect)(int jack_insert);
	int  (*codec_mic_switch)(bool on);
	
	void (*codec_set_spk)(bool on);
	void (*codec_set_hp)(bool on);
	void (*codec_mic_path)(enum mic_path path);
	void (*codec_sound_path)(enum sound_path path);
};

// for headset register
struct headset_external_fun {
	void (*hs_set_spk)(bool on);
	int (*hs_mic_path)(enum mic_path path);
	void (*hs_sound_path)(enum sound_path path);
};

extern struct codec_external_fun codec_ex_fun;
extern struct headset_external_fun headset_ex_fun;
int codec_ex_fun_register(struct codec_external_fun * codec_funs);
int headset_ex_fun_register(struct headset_external_fun * hs_funs);
struct codec_external_fun * get_codec_ex_fun(void);
struct headset_external_fun * get_headset_ex_fun(void);

#endif

