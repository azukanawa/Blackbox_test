cmake_minimum_required(VERSION 3.22)
enable_language(C ASM)

# 宏定义
set(MX_Defines_Syms 
	USE_HAL_DRIVER 
	STM32F103xB
    $<$<CONFIG:Debug>:DEBUG>
)

# 头文件路径
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
)

#自动加载 Core/Src 下所有 .c 文件
file(GLOB_RECURSE MX_Application_Src ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/*.c)
# 追加启动文件（必须单独加）
list(APPEND MX_Application_Src ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32f103xb.s)

# STM32 HAL驱动源码
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c
)

# 链接配置
set(MX_LINK_DIRS)
set(MX_LINK_LIBS STM32_Drivers ${TOOLCHAIN_LINK_LIBRARIES})


add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)

# 合并源码到工程
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# 编译优化
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()