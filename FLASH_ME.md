## Terminal A:

* `openocd -f interface/cmsis-dap.cfg -c "transport select swd"  -f target/stm32l4x.cfg`

## Terminal B:
* `telnet 127.0.0.1 4444`

### Telnet commands:

* `reset halt`

* `flash protect 0 0 last off`

* --POWER CYCLE--

* `reset halt`

* `stm32l4x option_write 0 0x20 0x0 0x0`

* -- POWER CYCLE --

* `reset halt`

* `stm32l4x unlock 0`

* -- POWER CYCLE -- 

* `reset halt`

* `stm32f4x mass erase 0`
