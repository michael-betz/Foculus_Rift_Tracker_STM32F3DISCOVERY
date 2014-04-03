# stop remaining openocd instances
killall -9 openocd
# start new openocd and reset and init chip
openocd -f /usr/local/share/openocd/scripts/board/stm32f3discovery.cfg -c init -c"reset init" -c"poll"
