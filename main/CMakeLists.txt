idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_timer esp_driver_gpio esp_driver_gptimer esp_driver_pcnt)