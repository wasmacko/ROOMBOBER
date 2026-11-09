idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS ".")
# Wyłączenie -Werror dla całego projektu lub komponentów
idf_build_set_property(COMPILE_OPTIONS "-Wno-error" APPEND)
