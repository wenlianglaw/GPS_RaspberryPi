ifconfig wlan0 | grep inet\  | awk '{print $2}' | xargs -I $ echo scp pi@$:`pwd`/paths.html ./
