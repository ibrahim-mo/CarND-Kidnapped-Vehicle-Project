#! /bin/bash
brew install openssl libuv cmake zlib
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig 
# bug fix -- Ibrahim
export OPENSSL_ROOT_DIR=/usr/local/opt/openssl
# another fix from forum
# export OPENSSL_ROOT_DIR=/usr/local/Cellar/openssl/1.0.2l
cd build
cmake ..
make 
sudo make install
cd ..
cd ..
sudo rm -r uWebSockets
