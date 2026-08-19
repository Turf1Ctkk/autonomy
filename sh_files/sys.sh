# Set CPU frequency to maximum
sudo echo "***** Set CPU frequency to maximum *****"
sudo echo "- Previous frequency of cores 1 2 3 4"
sudo cat /sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq
sudo echo "- Previous CPU governor of cores 1 2 3 4"
sudo cat /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
sudo echo

sudo echo "- Previous frequency of cores 5 6 7 8"
sudo cat /sys/devices/system/cpu/cpufreq/policy4/scaling_cur_freq
sudo echo "- Previous CPU governor of cores 5 6 7 8"
sudo cat /sys/devices/system/cpu/cpufreq/policy4/scaling_governor
sudo echo

sudo echo "- The list of available frequencies for cores 1 2 3 4"
sudo cat /sys/devices/system/cpu/cpufreq/policy0/scaling_available_frequencies

sudo echo "- The list of available frequencies for 5 6 7 8"
sudo cat /sys/devices/system/cpu/cpufreq/policy4/scaling_available_frequencies

sudo echo userspace > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
sudo echo 1984000 > /sys/devices/system/cpu/cpufreq/policy0/scaling_setspeed
sudo echo 1984000 > /sys/devices/system/cpu/cpufreq/policy0/scaling_max_freq

sudo echo userspace > /sys/devices/system/cpu/cpufreq/policy4/scaling_governor
sudo echo 1984000 > /sys/devices/system/cpu/cpufreq/policy4/scaling_setspeed
sudo echo 1984000 > /sys/devices/system/cpu/cpufreq/policy4/scaling_max_freq

sudo echo
sudo echo "- Changed frequency of cores 1 2 3 4"
sudo cat /sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq
sudo echo "- Changed CPU governor of cores 1 2 3 4"
sudo cat /sys/devices/system/cpu/cpufreq/policy0/scaling_governor

sudo echo "- Changed frequency of cores 5 6 7 8"
sudo cat /sys/devices/system/cpu/cpufreq/policy4/scaling_cur_freq
sudo echo "- Changed CPU governor of cores 5 6 7 8"
sudo cat /sys/devices/system/cpu/cpufreq/policy4/scaling_governor

sudo jetson_clocks & sleep 1;
sudo jetson_clocks --fan