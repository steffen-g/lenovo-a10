#define HID_MATRIX			1	// 0: scancode2, 1: hid


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

