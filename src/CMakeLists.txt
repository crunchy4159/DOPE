# Firmware component: only compile the application entry point.
# GUI and tooling sources live in tools/native_gui/ and are excluded here.

idf_component_register(SRCS "main.cpp")
