source ~/shaobing/devel/setup.zsh
{
    gnome-terminal --tab "listener_cmdvel" -- bash -c "roslaunch sentry_communication listener_cmdvel.launch"
}&
sleep 0.5s
{
    gnome-terminal --tab "listener_mcu" -- bash -c "roslaunch sentry_communication listener_mcu.launch"
}&
sleep 0.5s
{
    gnome-terminal --tab "publisher_mcu" -- bash -c "sudo chmod 666 /dev/ttyACM0;roslaunch sentry_communication publisher_mcu.launch"
}