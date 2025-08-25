#!/bin/bash

echo "🎯 M5Stack Tab5 Emulator - Firmware Test Script"
echo "================================================"

# Build the emulator if needed
echo "🔨 Building M5Stack Tab5 Emulator GUI..."
make m5tab5-emulator-gui -j$(nproc)

if [ $? -ne 0 ]; then
    echo "❌ Failed to build emulator GUI"
    exit 1
fi

echo "✅ Emulator built successfully"

# Check if firmware exists
if [ -f "firmware/m5tab5_test_firmware.cpp" ]; then
    echo "📁 Found test firmware: firmware/m5tab5_test_firmware.cpp"
    echo "📋 Firmware manifest: firmware/firmware_manifest.json"
else
    echo "❌ Test firmware not found"
    exit 1
fi

echo ""
echo "🚀 Launching M5Stack Tab5 Emulator with Test Firmware..."
echo "Expected behaviors:"
echo "  💡 LED blinking simulation"  
echo "  🔘 Button press detection"
echo "  🔗 I2C device scanning (BMI270, ES8388)"
echo "  📺 Display test patterns"
echo "  📊 System information logging"
echo ""
echo "Exit methods: ESC key, Q key, EXIT button, File->Exit, Window close"
echo ""

# Launch the emulator
./m5tab5-emulator-gui --firmware firmware/m5tab5_test_firmware.cpp

echo ""
echo "👋 Emulator session ended"