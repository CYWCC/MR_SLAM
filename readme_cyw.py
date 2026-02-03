
# 运行自采数据程序指令

1. /home/cyw/CYW/mapping/MR_SLAM/run_mrslam_docker.sh
 
2. docker exec -it mrslam bash /home/cyw_local/MR_SLAM/run_whu_mrslam_odom.sh run

3. 查看：docker exec -it mrslam tmux attach -t jackal_mrslam

# 播放前端里程计bag

1. --clock 需要添加在时间最长的bag后面，否则会运行不完整