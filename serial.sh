source ~/shaobing/devel/setup.bash
{
    gnome-terminal --tab "listener_cmdvel" -- bash -c "roslaunch new_serial listener_cmdvel.launch"
}&
sleep 2s
{
    gnome-terminal --tab "listener_mcu" -- bash -c "roslaunch new_serial listener_mcu.launch"
}&
sleep 2s
{
    gnome-terminal --tab "publisher_mcu" -- bash -c "sudo chmod 666 /dev/ttyACM0;roslaunch new_serial publisher_mcu.launch"
}