source ~/ws_livox/devel/setup.zsh
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCMAKE_EXPORT_COMPILE_COMMANDS=1
# catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_lio_localization"