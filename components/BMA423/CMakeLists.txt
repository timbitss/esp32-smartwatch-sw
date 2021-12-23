idf_component_register(SRCS "bma4.c" "bma423.c" "bma4_common.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES I2C
                       PRIV_REQUIRES I2C)