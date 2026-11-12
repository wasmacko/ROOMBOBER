idf_component_register(SRCS "src/main.cpp" "src/motor.cpp" "src/robot.cpp"
                    INCLUDE_DIRS "include")
# Wyłączenie -Werror dla całego projektu lub komponentów
idf_build_set_property(COMPILE_OPTIONS "-Wno-error" APPEND)
