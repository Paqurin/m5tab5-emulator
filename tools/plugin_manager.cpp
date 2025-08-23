#include "emulator/plugin/plugin_interface.hpp"
#include "emulator/utils/logging.hpp"
#include <iostream>
#include <filesystem>

using namespace m5tab5::emulator;

int main(int argc, char* argv[]) {
    std::cout << "M5Stack Tab5 Emulator - Plugin Manager\n";
    std::cout << "Version 1.0.0\n\n";
    
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <command> [options]\n\n";
        std::cout << "Commands:\n";
        std::cout << "  list [directory]    List available plugins\n";
        std::cout << "  info <plugin-path>  Show plugin information\n";
        std::cout << "  validate <plugin>   Validate plugin compatibility\n";
        std::cout << "  install <plugin>    Install plugin to system\n";
        std::cout << "  remove <plugin>     Remove installed plugin\n\n";
        std::cout << "Examples:\n";
        std::cout << "  " << argv[0] << " list plugins/\n";
        std::cout << "  " << argv[0] << " info plugins/example_peripheral.so\n";
        std::cout << "  " << argv[0] << " validate my_plugin.so\n";
        return 1;
    }
    
    std::string command = argv[1];
    
    if (command == "list") {
        std::string directory = (argc > 2) ? argv[2] : "plugins/";
        std::cout << "Searching for plugins in: " << directory << "\n";
        
        auto plugins = PluginLoader::findPlugins(directory);
        if (plugins.empty()) {
            std::cout << "No plugins found.\n";
        } else {
            std::cout << "Found " << plugins.size() << " plugin(s):\n";
            for (const auto& plugin : plugins) {
                std::cout << "  " << plugin << "\n";
            }
        }
    }
    else if (command == "info" && argc > 2) {
        std::string plugin_path = argv[2];
        std::cout << "Plugin information for: " << plugin_path << "\n";
        
        auto info = PluginLoader::getPluginInfo(plugin_path);
        if (!info.name.empty()) {
            std::cout << "  Name: " << info.name << "\n";
            std::cout << "  Version: " << info.version << "\n";
            std::cout << "  Author: " << info.author << "\n";
            std::cout << "  Description: " << info.description << "\n";
        } else {
            std::cout << "  Failed to load plugin information\n";
        }
    }
    else if (command == "validate" && argc > 2) {
        std::string plugin_path = argv[2];
        std::cout << "Validating plugin: " << plugin_path << "\n";
        
        bool valid = PluginLoader::validatePlugin(plugin_path);
        std::cout << "  Result: " << (valid ? "Valid" : "Invalid") << "\n";
    }
    else {
        std::cout << "Unknown command or missing arguments. Use --help for usage information.\n";
        return 1;
    }
    
    return 0;
}