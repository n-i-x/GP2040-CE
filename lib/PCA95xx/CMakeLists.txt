add_library(PCA95xx PCA95xx.h)
target_include_directories(PCA95xx INTERFACE .)
target_include_directories(PCA95xx PUBLIC .)
target_link_libraries(PCA95xx PUBLIC hardware_i2c)