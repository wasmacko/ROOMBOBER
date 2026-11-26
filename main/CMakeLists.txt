idf_component_register(
    SRCS
        "src/main.cpp"
        "src/robot.cpp"
        "src/motor.cpp"
        "src/bumper.cpp"
        "src/tof.cpp"
        "src/VL53L0X.cpp"
        "src/wifimanager.cpp"
        "src/webserver.cpp"
        "src/collision.cpp"
        "src/controller.cpp"
        "src/odometry.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_wifi
        esp_http_server
        nvs_flash
        esp_netif
        esp_driver_pcnt
)
# Wyłączenie -Werror dla całego projektu lub komponentów
idf_build_set_property(COMPILE_OPTIONS "-Wno-error" APPEND)