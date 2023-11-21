cd build
cmake .. -GNinja
cmake --build .
cd ..
picotool reboot -f -u
sleep 2
picotool load ./build/scioly_bot.uf2
picotool reboot
