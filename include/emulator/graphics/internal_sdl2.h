#pragma once

// Minimal SDL2 declarations for basic window functionality
// Used when SDL2 development headers are not available but library is present

#ifdef INTERNAL_SDL2_HEADERS

#ifdef __cplusplus
extern "C" {
#endif

// Basic types
typedef struct SDL_Window SDL_Window;
typedef struct SDL_Renderer SDL_Renderer;  
typedef struct SDL_Texture SDL_Texture;
typedef unsigned int Uint32;
typedef unsigned short Uint16;
typedef unsigned char Uint8;
typedef signed int Sint32;
typedef signed short Sint16;

// Audio types
typedef Uint32 SDL_AudioDeviceID;
typedef Uint16 SDL_AudioFormat;

typedef struct {
    int freq;
    SDL_AudioFormat format;
    Uint8 channels;
    Uint8 silence;
    Uint16 samples;
    Uint16 padding;
    Uint32 size;
    void (*callback)(void *userdata, Uint8 *stream, int len);
    void *userdata;
} SDL_AudioSpec;

// SDL_Init flags
#define SDL_INIT_VIDEO 0x00000020u
#define SDL_INIT_TIMER 0x00000001u

// Window flags
#define SDL_WINDOWPOS_CENTERED 0x2FFF0000u
#define SDL_WINDOW_SHOWN 0x00000004
#define SDL_WINDOW_RESIZABLE 0x00000020

// Renderer flags
#define SDL_RENDERER_ACCELERATED 0x00000002
#define SDL_RENDERER_PRESENTVSYNC 0x00000004

// Pixel formats
#define SDL_PIXELFORMAT_RGBA8888 0x16462004

// Texture access
#define SDL_TEXTUREACCESS_STREAMING 1

// Event types
#define SDL_QUIT 0x100
#define SDL_KEYDOWN 0x300
#define SDL_WINDOWEVENT 0x200
#define SDL_WINDOWEVENT_CLOSE 1
#define SDL_MOUSEBUTTONDOWN 0x401
#define SDL_MOUSEBUTTONUP 0x402
#define SDL_MOUSEMOTION 0x400

// Mouse button constants
#define SDL_BUTTON_LEFT 1
#define SDL_BUTTON_MIDDLE 2
#define SDL_BUTTON_RIGHT 3

// Mouse button mask macro
#define SDL_BUTTON(X) (1 << ((X)-1))

// Key codes (SDL2 standard key codes)
#define SDLK_ESCAPE 27
#define SDLK_SPACE 32
#define SDLK_RETURN 13
#define SDLK_a 97
#define SDLK_b 98
#define SDLK_c 99
#define SDLK_l 108
#define SDLK_o 111
#define SDLK_q 113
#define SDLK_r 114
#define SDLK_s 115
#define SDLK_v 118
#define SDLK_y 121
#define SDLK_z 122
#define SDLK_UP 1073741906
#define SDLK_DOWN 1073741905
#define SDLK_LEFT 1073741904
#define SDLK_RIGHT 1073741903
#define SDLK_F1 1073741882
#define SDLK_F2 1073741883
#define SDLK_F3 1073741884
#define SDLK_F4 1073741885
#define SDLK_F5 1073741886
#define SDLK_F6 1073741887
#define SDLK_F7 1073741888
#define SDLK_F8 1073741889
#define SDLK_F9 1073741890
#define SDLK_F10 1073741891
#define SDLK_F11 1073741892
#define SDLK_F12 1073741893

// Modifier keys
#define KMOD_CTRL 0x0040
#define KMOD_SHIFT 0x0003
#define KMOD_ALT 0x0100

// Event structures
typedef struct {
    Uint32 type;
    Uint32 timestamp;
} SDL_CommonEvent;

typedef struct {
    Sint32 sym;
    Uint32 mod;
} SDL_Keysym;

typedef struct {
    Uint32 type;
    Uint32 timestamp;
    Uint32 windowID;
    SDL_Keysym keysym;
} SDL_KeyboardEvent;

typedef struct {
    Uint32 type;
    Uint32 timestamp;
    Uint32 windowID;
    Uint8 event;
} SDL_WindowEvent;

typedef struct {
    Uint32 type;
    Uint32 timestamp;
    Uint32 windowID;
    Uint8 button;
    Uint8 state;
    Uint8 clicks;
    Uint8 padding1;
    Sint32 x;
    Sint32 y;
} SDL_MouseButtonEvent;

typedef struct {
    Uint32 type;
    Uint32 timestamp;
    Uint32 windowID;
    Uint32 which;
    Uint32 state;
    Sint32 x;
    Sint32 y;
    Sint32 xrel;
    Sint32 yrel;
} SDL_MouseMotionEvent;

typedef union {
    Uint32 type;
    SDL_CommonEvent common;
    SDL_KeyboardEvent key;
    SDL_WindowEvent window;
    SDL_MouseButtonEvent button;
    SDL_MouseMotionEvent motion;
} SDL_Event;

// Function declarations
int SDL_Init(Uint32 flags);
void SDL_Quit(void);
const char* SDL_GetError(void);

SDL_Window* SDL_CreateWindow(const char* title, int x, int y, int w, int h, Uint32 flags);
void SDL_DestroyWindow(SDL_Window* window);

SDL_Renderer* SDL_CreateRenderer(SDL_Window* window, int index, Uint32 flags);
void SDL_DestroyRenderer(SDL_Renderer* renderer);
int SDL_RenderClear(SDL_Renderer* renderer);
int SDL_RenderCopy(SDL_Renderer* renderer, SDL_Texture* texture, const void* srcrect, const void* dstrect);
void SDL_RenderPresent(SDL_Renderer* renderer);
int SDL_SetRenderDrawColor(SDL_Renderer* renderer, Uint8 r, Uint8 g, Uint8 b, Uint8 a);
int SDL_RenderFillRect(SDL_Renderer* renderer, const void* rect);

// SDL_Rect structure
typedef struct {
    Sint32 x, y;
    Sint32 w, h;
} SDL_Rect;

SDL_Texture* SDL_CreateTexture(SDL_Renderer* renderer, Uint32 format, int access, int w, int h);
void SDL_DestroyTexture(SDL_Texture* texture);
int SDL_UpdateTexture(SDL_Texture* texture, const void* rect, const void* pixels, int pitch);

int SDL_PollEvent(SDL_Event* event);

// Mouse state functions
Uint32 SDL_GetMouseState(int* x, int* y);

#ifdef __cplusplus
}
#endif

#endif // INTERNAL_SDL2_HEADERS