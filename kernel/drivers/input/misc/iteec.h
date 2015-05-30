#ifndef _ASUSDEC_H
#define _ASUSDEC_H

#include <linux/switch.h>
#include <linux/wakelock.h>

/*
 * compiler option
 */
#define ASUSDEC_DEBUG               0
#define TOUCHPAD_MODE               0   // 0: relative mode, 1: absolute mode

#define TOUCHPAD_ELAN               1   // 0: not elan, 1:elantech
#define MY_DBG                      1

#define HID_MATRIX					1	// 0: scancode2, 1: hid

#define ASUSDEC_OBF_MASK        0x1
#define ASUSDEC_KEY_MASK        0x4
#define ASUSDEC_KBC_MASK        0x8
#define ASUSDEC_AUX_MASK        0x20
#define ASUSDEC_SCI_MASK        0x40
#define ASUSDEC_SMI_MASK        0x80

/* relative mode packet formate */
#define Y_OVERFLOW_MASK         0x80
#define X_OVERFLOW_MASK         0x40
#define Y_SIGN_MASK             0x20
#define X_SIGN_MASK             0x10
#define RIGHT_BTN_MASK          0x2
#define LEFT_BTN_MASK           0x1

/*
 * Debug Utility
 */
#if ASUSDEC_DEBUG
#define ASUSDEC_INFO(format, arg...)    \
    printk(KERN_INFO "DOCKEC: [%s] " format , __FUNCTION__ , ## arg)
#define ASUSDEC_I2C_DATA(array, i)  \
                    do {        \
                        for (i = 0; i < array[0]+1; i++) \
                            ASUSDEC_INFO("ec_data[%d] = 0x%x\n", i, array[i]);  \
                    } while(0)
#else
#define ASUSDEC_INFO(format, arg...)     
#define ASUSDEC_I2C_DATA(array, i)
#endif

#define ASUSDEC_NOTICE(format, arg...)  \
    printk(KERN_NOTICE "DOCKEC: [%s] " format , __FUNCTION__ , ## arg)

#define ASUSDEC_ERR(format, arg...) \
    printk(KERN_ERR "DOCKEC: [%s] " format , __FUNCTION__ , ## arg)

/////////////////////////////////////////////////////////////
#if	HID_MATRIX

#define ASUSDEC_KEYPAD_ESC              0x29
#define ASUSDEC_KEYPAD_KEY_WAVE         0x35// `~
#define ASUSDEC_KEYPAD_KEY_1            0x1e
#define ASUSDEC_KEYPAD_KEY_2            0X1f
#define ASUSDEC_KEYPAD_KEY_3            0x20    
#define ASUSDEC_KEYPAD_KEY_4            0x21
#define ASUSDEC_KEYPAD_KEY_5            0x22
#define ASUSDEC_KEYPAD_KEY_6            0x23
#define ASUSDEC_KEYPAD_KEY_7            0x24
#define ASUSDEC_KEYPAD_KEY_8            0x25
#define ASUSDEC_KEYPAD_KEY_9            0x26
#define ASUSDEC_KEYPAD_KEY_0            0x27
#define ASUSDEC_KEYPAD_KEY_MINUS        0x2d// -_
#define ASUSDEC_KEYPAD_KEY_EQUAL        0x2e// =+
#define ASUSDEC_KEYPAD_KEY_BACKSPACE    0x2a
#define ASUSDEC_KEYPAD_KEY_TAB          0x2b
#define ASUSDEC_KEYPAD_KEY_Q            0x14
#define ASUSDEC_KEYPAD_KEY_W            0x1a
#define ASUSDEC_KEYPAD_KEY_E            0x08
#define ASUSDEC_KEYPAD_KEY_R            0x15
#define ASUSDEC_KEYPAD_KEY_T            0x17
#define ASUSDEC_KEYPAD_KEY_Y            0x1c
#define ASUSDEC_KEYPAD_KEY_U            0x18
#define ASUSDEC_KEYPAD_KEY_I            0x0c
#define ASUSDEC_KEYPAD_KEY_O            0x12
#define ASUSDEC_KEYPAD_KEY_P            0x13
#define ASUSDEC_KEYPAD_KEY_LEFTBRACE    0x2f// [{
#define ASUSDEC_KEYPAD_KEY_RIGHTBRACE   0x30// ]}
#define ASUSDEC_KEYPAD_KEY_BACKSLASH    0x31// \|
#define ASUSDEC_KEYPAD_KEY_CAPSLOCK     0x39
#define ASUSDEC_KEYPAD_KEY_A            0x04
#define ASUSDEC_KEYPAD_KEY_S            0x16
#define ASUSDEC_KEYPAD_KEY_D            0x07
#define ASUSDEC_KEYPAD_KEY_F            0x09
#define ASUSDEC_KEYPAD_KEY_G            0x0a
#define ASUSDEC_KEYPAD_KEY_H            0x0b
#define ASUSDEC_KEYPAD_KEY_J            0x0d
#define ASUSDEC_KEYPAD_KEY_K            0x0e
#define ASUSDEC_KEYPAD_KEY_L            0x0f
#define ASUSDEC_KEYPAD_KEY_SEMICOLON    0x33// ;:
#define ASUSDEC_KEYPAD_KEY_APOSTROPHE   0x34// '"
#define ASUSDEC_KEYPAD_KEY_ENTER        0x28
#define ASUSDEC_KEYPAD_KEY_LEFTSHIFT    0xe1
#define ASUSDEC_KEYPAD_KEY_Z            0x1d
#define ASUSDEC_KEYPAD_KEY_X            0x1b
#define ASUSDEC_KEYPAD_KEY_C            0x06
#define ASUSDEC_KEYPAD_KEY_V            0x19
#define ASUSDEC_KEYPAD_KEY_B            0x05
#define ASUSDEC_KEYPAD_KEY_N            0x11
#define ASUSDEC_KEYPAD_KEY_M            0x10
#define ASUSDEC_KEYPAD_KEY_COMMA        0x36// ,<
#define ASUSDEC_KEYPAD_KEY_DOT          0x37// .>
#define ASUSDEC_KEYPAD_KEY_SLASH        0x38// ?/
#define ASUSDEC_KEYPAD_KEY_RIGHTSHIFT   0xe5

#define ASUSDEC_KEYPAD_KEY_LEFT         0x50
#define ASUSDEC_KEYPAD_KEY_RIGHT        0x4f
#define ASUSDEC_KEYPAD_KEY_UP           0x52
#define ASUSDEC_KEYPAD_KEY_DOWN         0x51
#define ASUSDEC_KEYPAD_RIGHTWIN         0xe7// Right GUI
#define ASUSDEC_KEYPAD_LEFTCTRL         0xe0
#define ASUSDEC_KEYPAD_LEFTWIN          0xe3// Left GUI
#define ASUSDEC_KEYPAD_LEFTALT          0xe2
#define ASUSDEC_KEYPAD_KEY_SPACE        0x2c
#define ASUSDEC_KEYPAD_RIGHTALT         0xe6
#define ASUSDEC_KEYPAD_WINAPP           0x65
#define ASUSDEC_KEYPAD_RIGHTCTRL        0xe4
#define ASUSDEC_KEYPAD_HOME             0x4a
#define ASUSDEC_KEYPAD_PAGEUP           0x4b
#define ASUSDEC_KEYPAD_PAGEDOWN         0x4e
#define ASUSDEC_KEYPAD_END              0x4d

#define ASUSDEC_KEYPAD_F1				0x3a
#define ASUSDEC_KEYPAD_F2				0x3b
#define ASUSDEC_KEYPAD_F3				0x3c
#define ASUSDEC_KEYPAD_F4				0x3d
#define ASUSDEC_KEYPAD_F5				0x3e
#define ASUSDEC_KEYPAD_F6				0x3f
#define ASUSDEC_KEYPAD_F7				0x40
#define ASUSDEC_KEYPAD_F8				0x41
#define ASUSDEC_KEYPAD_F9				0x42
#define ASUSDEC_KEYPAD_F10				0x43
#define ASUSDEC_KEYPAD_F11				0x44
#define ASUSDEC_KEYPAD_F12				0x45
#define ASUSDEC_KEYPAD_INSERT			0x49
#define ASUSDEC_KEYPAD_PRINTSCREEN		0x46
#define ASUSDEC_KEYPAD_DELETE			0x4c

//OEM KEY
#define KEY_LAUNCHER		250
#define KEY_SETTING		251
#define KEY_TOUCHPAD		252
#define KEY_FORCE_ROTATION	253
#define KEY_SCREENSHOT		254

/************  JP keys *************/
//#define ASUSDEC_HANKAKU_ZENKAKU         0x5F                
//#define ASUSDEC_YEN                     0x6A
//#define ASUSDEC_MUHENKAN                0x67        
//#define ASUSDEC_HENKAN                  0x64        
//#define ASUSDEC_HIRAGANA_KATAKANA       0x13
//#define ASUSDEC_RO                      0x51
/********************************/
/************  UK keys *************/
//#define ASUSDEC_EUROPE_2                0x61
/********************************/

#else

/*************scan 2 make mapping***************/
#define ASUSDEC_KEYPAD_ESC              0x76
#define ASUSDEC_KEYPAD_KEY_WAVE         0x0E
#define ASUSDEC_KEYPAD_KEY_1            0x16
#define ASUSDEC_KEYPAD_KEY_2            0X1E
#define ASUSDEC_KEYPAD_KEY_3            0x26    
#define ASUSDEC_KEYPAD_KEY_4            0x25
#define ASUSDEC_KEYPAD_KEY_5            0x2E
#define ASUSDEC_KEYPAD_KEY_6            0x36
#define ASUSDEC_KEYPAD_KEY_7            0x3D
#define ASUSDEC_KEYPAD_KEY_8            0x3E
#define ASUSDEC_KEYPAD_KEY_9            0x46
#define ASUSDEC_KEYPAD_KEY_0            0x45
#define ASUSDEC_KEYPAD_KEY_MINUS        0x4E
#define ASUSDEC_KEYPAD_KEY_EQUAL        0x55
#define ASUSDEC_KEYPAD_KEY_BACKSPACE    0x66
#define ASUSDEC_KEYPAD_KEY_TAB          0x0D
#define ASUSDEC_KEYPAD_KEY_Q            0x15
#define ASUSDEC_KEYPAD_KEY_W            0x1D
#define ASUSDEC_KEYPAD_KEY_E            0x24
#define ASUSDEC_KEYPAD_KEY_R            0x2D
#define ASUSDEC_KEYPAD_KEY_T            0x2C
#define ASUSDEC_KEYPAD_KEY_Y            0x35
#define ASUSDEC_KEYPAD_KEY_U            0x3C
#define ASUSDEC_KEYPAD_KEY_I            0x43
#define ASUSDEC_KEYPAD_KEY_O            0x44
#define ASUSDEC_KEYPAD_KEY_P            0x4D
#define ASUSDEC_KEYPAD_KEY_LEFTBRACE    0x54
#define ASUSDEC_KEYPAD_KEY_RIGHTBRACE   0x5B
#define ASUSDEC_KEYPAD_KEY_BACKSLASH    0x5D
#define ASUSDEC_KEYPAD_KEY_CAPSLOCK     0x58
#define ASUSDEC_KEYPAD_KEY_A            0x1C
#define ASUSDEC_KEYPAD_KEY_S            0x1B
#define ASUSDEC_KEYPAD_KEY_D            0x23
#define ASUSDEC_KEYPAD_KEY_F            0x2B
#define ASUSDEC_KEYPAD_KEY_G            0x34
#define ASUSDEC_KEYPAD_KEY_H            0x33
#define ASUSDEC_KEYPAD_KEY_J            0x3B
#define ASUSDEC_KEYPAD_KEY_K            0x42
#define ASUSDEC_KEYPAD_KEY_L            0x4B
#define ASUSDEC_KEYPAD_KEY_SEMICOLON    0x4C
#define ASUSDEC_KEYPAD_KEY_APOSTROPHE   0x52
#define ASUSDEC_KEYPAD_KEY_ENTER        0x5A
#define ASUSDEC_KEYPAD_KEY_LEFTSHIFT    0x12
#define ASUSDEC_KEYPAD_KEY_Z            0x1A
#define ASUSDEC_KEYPAD_KEY_X            0x22
#define ASUSDEC_KEYPAD_KEY_C            0x21
#define ASUSDEC_KEYPAD_KEY_V            0x2A
#define ASUSDEC_KEYPAD_KEY_B            0x32
#define ASUSDEC_KEYPAD_KEY_N            0x31
#define ASUSDEC_KEYPAD_KEY_M            0x3A
#define ASUSDEC_KEYPAD_KEY_COMMA        0x41
#define ASUSDEC_KEYPAD_KEY_DOT          0x49
#define ASUSDEC_KEYPAD_KEY_SLASH        0x4A
#define ASUSDEC_KEYPAD_KEY_RIGHTSHIFT   0x59

#define ASUSDEC_KEYPAD_KEY_LEFT         0xE06B
#define ASUSDEC_KEYPAD_KEY_RIGHT        0xE074
#define ASUSDEC_KEYPAD_KEY_UP           0xE075
#define ASUSDEC_KEYPAD_KEY_DOWN         0xE072
#define ASUSDEC_KEYPAD_RIGHTWIN         0xE027
#define ASUSDEC_KEYPAD_LEFTCTRL         0x14
#define ASUSDEC_KEYPAD_LEFTWIN          0xE01F
#define ASUSDEC_KEYPAD_LEFTALT          0x11
#define ASUSDEC_KEYPAD_KEY_SPACE        0x29
#define ASUSDEC_KEYPAD_RIGHTALT         0xE011
#define ASUSDEC_KEYPAD_WINAPP           0xE02F
#define ASUSDEC_KEYPAD_RIGHTCTRL        0xE014
#define ASUSDEC_KEYPAD_HOME             0xE06C
#define ASUSDEC_KEYPAD_PAGEUP           0xE07D
#define ASUSDEC_KEYPAD_PAGEDOWN         0xE07A
#define ASUSDEC_KEYPAD_END              0xE069
/************  JP keys *************/
#define ASUSDEC_HANKAKU_ZENKAKU         0x5F                
#define ASUSDEC_YEN                     0x6A
#define ASUSDEC_MUHENKAN                0x67        
#define ASUSDEC_HENKAN                  0x64        
#define ASUSDEC_HIRAGANA_KATAKANA       0x13
#define ASUSDEC_RO                      0x51
/********************************/
/************  UK keys *************/
#define ASUSDEC_EUROPE_2                0x61
/********************************/

#endif

#define ASUSDEC_KEYPAD_LOCK             0xE071

#define ASUSDEC_KEYPAD_KEY_BREAK        0xF0
#define ASUSDEC_KEYPAD_KEY_EXTEND       0xE0

/////////////////////////////////////////////////////////////

/*************IO control setting***************/
#define ASUSDEC_IOCTL_HEAVY     2
#define ASUSDEC_IOCTL_NORMAL    1
#define ASUSDEC_IOCTL_END       0
#define ASUSDEC_CPAS_LED_ON     1
#define ASUSDEC_CPAS_LED_OFF    0
#define ASUSDEC_TP_ON           1
#define ASUSDEC_TP_OFF          0
#define ASUSDEC_EC_ON           1
#define ASUSDEC_EC_OFF          0
#define ASUSDEC_IOC_MAGIC       0xf4
#define ASUSDEC_IOC_MAXNR       7
#define ASUSDEC_POLLING_DATA    _IOR(ASUSDEC_IOC_MAGIC, 1,  int)
#define ASUSDEC_FW_UPDATE       _IOR(ASUSDEC_IOC_MAGIC, 2,  int)
#define ASUSDEC_CPASLOCK_LED    _IOR(ASUSDEC_IOC_MAGIC, 3,  int)
#define ASUSDEC_INIT            _IOR(ASUSDEC_IOC_MAGIC, 4,  int)
#define ASUSDEC_TP_CONTROL      _IOR(ASUSDEC_IOC_MAGIC, 5,  int)
#define ASUSDEC_EC_WAKEUP       _IOR(ASUSDEC_IOC_MAGIC, 6,  int)
#define ASUSDEC_FW_DUMMY        _IOR(ASUSDEC_IOC_MAGIC, 7,  int)
/*************IO control setting***************/

/************* EC FW update ***********/
#define EC_BUFF_LEN             256
/********************** ***********/

struct asusdec_keypad{
    int value;
    int input_keycode;
    int extend;
};

struct asusdec_touchpad_relative{
    int y_overflow;
    int x_overflow;
    int y_sign;
    int x_sign;
    int left_btn;
    int right_btn;
    int delta_x;
    int delta_y;
};

struct asusdec_touchpad_absolute{
    int w_val;
    int x_pos;
    int y_pos;
    int z_val;
    int left;
    int right;
    int x_prev;
    int y_prev;
    int z_prev;
    int x2_pos;
    int y2_pos;
    int z2_val;
};

struct elantech_data {
    unsigned int xmax;
    unsigned int ymax;
    unsigned int fw_version;

    int left_button;
    int right_button;

    int fingers;
    struct { int x, y; } pos[2];

    struct input_dev *abs_dev;
};

struct asusdec_chip {
    struct input_dev    *indev;
    struct input_dev    *lid_indev;
    struct switch_dev   dock_sdev;
    struct i2c_client   *client;
    struct mutex        lock;
    struct mutex        kbc_lock;
    struct mutex        input_lock;
    struct mutex        dock_init_lock;
    struct wake_lock    wake_lock;
    struct wake_lock    wake_lock_timeout;
    struct wake_lock    wake_lock_init;
    struct delayed_work asusdec_work;
    struct delayed_work asusdec_dock_init_work;
    struct delayed_work asusdec_fw_update_work;
    struct delayed_work asusdec_led_on_work;
    struct delayed_work asusdec_led_off_work;
    struct delayed_work asusdec_hall_sensor_work;
//  struct delayed_work asusdec_audio_report_work;
    struct delayed_work asusdec_battery_poll_data_work;
    struct delayed_work asusdec_status_to_accessory_work;
    struct asusdec_keypad keypad_data;
    struct elantech_data *private;
    struct timer_list asusdec_timer;
#if TOUCHPAD_MODE   
    struct asusdec_touchpad_absolute t_abs;
#else
    struct asusdec_touchpad_relative touchpad_data;
#endif
    struct early_suspend asusdec_early_suspend;

    int ret_val;
    u8 ec_data[32];
    u8 i2c_data[32];
    u8 i2c_dm_data[32];
    int bc;                 // byte counter
    int index;              // for message
    int status;
    int touchpad_member;
    char ec_model_name[32];
    char ec_version[32];
    char ec_config_format[32];
    char dock_pid[32];
    char dcok_board_ec_version[32];
    int polling_rate;
    int dock_in;            // 0: without dock, 1: with dock
    int op_mode;            // 0: normal mode, 1: fw update mode    
    int kbc_value;          // capslock_led 0: led off, 1: led on
    int dock_det;           // dock-in interrupt count
    int dock_init;          // 0: dock not init, 1: dock init successfully
    int d_index;            // touchpad byte counter
    int suspend_state;      // 0: normal, 1: suspend
    int init_success;       // 0: ps/2 not ready. 1: init OK, -1: tp not ready
    int wakeup_lcd;         // 0 : keep lcd state 1: make lcd on
    int tp_wait_ack;        // 0 : normal mode, 1: waiting for an ACK
    int tp_enable;          // 0 : touchpad has not enabled, 1: touchpad has enabled
    int re_init;            // 0 : first time init, not re-init, 1: in re-init procedure
    int ec_wakeup;          // 0 : ec shutdown when PAD in LP0, 1 : keep ec active when PAD in LP0,
    int ap_wake_wakeup;     // 0 : no ap_wake wakeup signal, 1: get ap_wake wakeup signal
    int tf201_dock;         // 0 : not tf201 dock, 1: tf201 dock
    int dock_behavior;      // 0: susb_on follows wakeup event, 1: susb_on follows ec_req
    int ec_in_s3;           // 0: normal mode, 1: ec in deep sleep mode
    int susb_on;            // 0: susb off, 1: susb on
    int hall_sensor_status; // 0 : active, 1: Sub-cam standby
    int docking_status;     // 0: docking not ok, 1: docking status is ok
    int kbc_value_backup;   // suspend status: capslock_led 0: led off, 1: led on

    //EC read ++
    int ec_smbus_status;
    int ec_bat_voltage;
    int ec_bat_current;
    int ec_bat_capacity;    //percentage
    int ec_bat_status;
    int ec_bat_temp;
    //EC read --
    //return to system ++
    int bat_status;
    int bat_temperature;
    int bat_percentage;     //capacity
    int bat_capacity_level;
    int bat_voltage;
    int bat_current;
    //return to system ++
    //storage old data ++
    int old_bat_status;
    int old_bat_temperature;
    int old_bat_percentage;     //capacity
    int old_bat_voltage;
    int old_bat_current;
    //storage old data --
};

#endif
