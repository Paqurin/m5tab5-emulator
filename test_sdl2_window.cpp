#include "include/emulator/graphics/internal_sdl2.h"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    std::cout << "Testing SDL2 window creation with M5Stack Tab5 resolution..." << std::endl;
    
    // Initialize SDL2
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl;
        return 1;
    }
    
    std::cout << "SDL2 initialized successfully!" << std::endl;
    
    // Create window with M5Stack Tab5 resolution (1280x720)
    SDL_Window* window = SDL_CreateWindow(
        "M5Stack Tab5 Emulator - Professional Development GUI",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        1280, 720,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );
    
    if (!window) {
        std::cerr << "Failed to create SDL window: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }
    
    std::cout << "SDL2 window created successfully at 1280x720!" << std::endl;
    
    // Create renderer
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        std::cerr << "Failed to create SDL renderer: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    
    std::cout << "SDL2 renderer created successfully!" << std::endl;
    std::cout << "Window should now be visible. Running for 10 seconds..." << std::endl;
    
    // Event loop for 10 seconds
    auto start_time = std::chrono::steady_clock::now();
    bool running = true;
    
    while (running) {
        // Handle events
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
                break;
            }
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) {
                running = false;
                break;
            }
        }
        
        // Check if 10 seconds have passed
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
        if (elapsed.count() >= 10) {
            running = false;
        }
        
        // Clear screen with M5Stack orange-themed color
        SDL_SetRenderDrawColor(renderer, 255, 102, 0, 255); // M5Stack orange
        SDL_RenderClear(renderer);
        
        // Present frame
        SDL_RenderPresent(renderer);
        
        // Small delay
        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }
    
    std::cout << "Test completed successfully!" << std::endl;
    
    // Cleanup
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    
    std::cout << "SDL2 window test completed - Window should have been visible!" << std::endl;
    return 0;
}