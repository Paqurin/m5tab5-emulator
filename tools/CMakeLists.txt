# Tools and utilities for M5Stack Tab5 Emulator

# Configuration tool
add_executable(m5tab5-config
    config_tool.cpp
)
target_link_libraries(m5tab5-config
    PRIVATE m5tab5-emulator-core
)

# Memory inspector tool
add_executable(m5tab5-memory-inspector
    memory_inspector.cpp
)
target_link_libraries(m5tab5-memory-inspector
    PRIVATE m5tab5-emulator-core
)

# Plugin manager tool (temporarily disabled)
# add_executable(m5tab5-plugin-manager
#     plugin_manager.cpp
# )
# target_link_libraries(m5tab5-plugin-manager
#     PRIVATE m5tab5-plugin-support
# )

# Install tools
install(TARGETS m5tab5-config m5tab5-memory-inspector
    RUNTIME DESTINATION bin
)