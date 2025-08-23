# Plugin examples for M5Stack Tab5 Emulator
# These are built separately and dynamically loaded

# Example peripheral plugin
add_library(example_peripheral_plugin SHARED
    example_peripheral.cpp
)
target_link_libraries(example_peripheral_plugin 
    PRIVATE m5tab5-emulator-core
)

# Example analyzer plugin  
add_library(example_analyzer_plugin SHARED
    example_analyzer.cpp
)
target_link_libraries(example_analyzer_plugin 
    PRIVATE m5tab5-emulator-core
)

# Install plugins to lib/m5tab5-emulator/plugins
install(TARGETS example_peripheral_plugin example_analyzer_plugin
    LIBRARY DESTINATION lib/m5tab5-emulator/plugins
)