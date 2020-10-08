MESSAGE="Installing BarrettHand Dependencies: codeblocks build-essential freeglut3-dev libpopt-dev libpoco-dev wget..." && \
sudo apt-get update > /dev/null
sudo apt-get install -y build-essential freeglut3-dev libpopt-dev libpoco-dev wget > /dev/null

#
MESSAGE="Download and install the PCAN Driver for BarrettHand..." && \
wget https://auth-arl.github.io/deps/peak-linux-driver-8.5.1.tar.gz
tar xvf peak-linux-driver-8.5.1.tar.gz > /dev/null
cd peak-linux-driver-8.5.1
make
sudo make install
sudo modprobe pcan
cd ..
rm -rf peak-linux-driver-8.5.1.tar.gz
rm -rf peak-linux-driver-8.5.1
cd ..

if [ $? -eq 0 ]; then
  MESSAGE="BarrettHand drivers successfully installed."
  cd ..
else
  echo "BarrettHand drivers failed to install."
  cd ..
  exit 1
fi
