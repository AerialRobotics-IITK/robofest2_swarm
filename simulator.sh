# git clone https://github.com/AerialRobotics-IITK/robofest2_swarm.git 
cd ~/robofest
gazebo --verbose ~/robofest/robofest_gazebo/worlds/robofest_sim.world
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
chmod +x QGroundControl.AppImage
./QGroundControl
sudo apt install xterm
xterm -e bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris -I1 --out=tcpin:0.0.0.0:8100;" &
xterm -e bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris -I2 --out=tcpin:0.0.0.0:8200;" &
xterm -e bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris -I3 --out=tcpin:0.0.0.0:8300;" &
xterm -e bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris -I4 --out=tcpin:0.0.0.0:8400;" &
xterm -e bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris -I5 --out=tcpin:0.0.0.0:8500;" &c