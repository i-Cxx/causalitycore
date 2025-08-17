# Dies ist die CMakeLists.txt für Ihren Anwendungs-Code (main.c etc.).

# Erstellt das ausführbare Programm namens 'fusioncoreos'.
# Listet NUR die Quelldateien auf, die DIREKT zu Ihrer HAUPTANWENDUNG gehören.
# KEINE Quelldateien von Bibliotheken wie lcd_1602_i2c.c oder ssd1306_i2c.c hier auflisten!
add_executable(causalitycore
    main.c
    # Fügen Sie hier weitere .c/.cpp-Dateien hinzu, die Sie selbst für Ihre Anwendung geschrieben haben
    # Beispiel: my_app_logic.c
)

target_include_directories(causalitycore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp
)


# Verknüpfen Sie Ihr ausführbares Programm mit allen benötigten Bibliotheken.
target_link_libraries(causalitycore
    pico_stdlib           
    pico_binary_info     
    hardware_gpio        
    hardware_i2c  
    hardware_adc       
    freertos              

    lcd_1602_i2c_lib      
    ssd1306_i2c_lib       
    blink_lib
)

pico_enable_stdio_usb(causalitycore 1)   # USB einschalten
pico_enable_stdio_uart(causalitycore 0) # UART ausschalten

# Diese Funktion des Pico SDK ist essentiell, um die notwendigen Ausgabedateien
# (wie .uf2) für den Raspberry Pi Pico zu generieren.
pico_add_extra_outputs(causalitycore)