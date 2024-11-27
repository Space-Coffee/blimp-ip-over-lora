cmake_minimum_required(VERSION 3.2)

project(blimp-ip-over-lora)

add_executable(blimp-ip-over-lora
	src/main.c
	src/spi.c
	src/gpio.c
	src/sx1278.c)

