idf_component_register(SRCS "esp32_smartwatch.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES BMA423 RTClib Buzzer ButtonDebouncer
                       PRIV_REQUIRES BMA423 RTClib Buzzer ButtonDebouncer)
