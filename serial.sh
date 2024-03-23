source ~/shaobing/devel/setup.zsh
{
    gnome-terminal --tab "listener_cmdvel" -- zsh -c "roslaunch sentry_communication listener_cmdvel.launch; exec zsh"
}&
sleep 0.5s
{
    gnome-terminal --tab "listener_mcu" -- zsh -c "roslaunch sentry_communication listener_mcu.launch; exec zsh"
}&
sleep 0.5s
{
    gnome-terminal --tab "publisher_mcu" -- zsh -c "roslaunch sentry_communication publisher_mcu.launch; exec zsh"
}