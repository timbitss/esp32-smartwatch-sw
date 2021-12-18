idf_component_register(SRCS "RTClib.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES I2C
                       PRIV_REQUIRES I2C)