
# 运行自采数据程序指令

1. /home/cyw/CYW/mapping/MR_SLAM/run_mrslam_docker.sh
 
2. docker exec -it mrslam bash /home/cyw_local/MR_SLAM/run_whu_mrslam_odom.sh run

3. 查看:docker exec -it mrslam tmux attach -t jackal_mrslam

# 播放前端里程计bag

1. --clock 需要添加在时间最长的bag后面,否则会运行不完整

2. 参数

SC 参数：dist_thresh  /  icp_fitness_score
group5:  0.4, 0.6
group6，7:  0.25, 0.3
group1: 0.25 0.2
snail: 0.6, 0.3
S3E_Playground_1: 0.6, 0.3
S3E_Square_1:

Disco 参数：dist_thresh  /  icp_fitness_score
group5,6:  8.0, 0.2
group7: 运行会中断，有问题
group1: 5.0, 0.2
snail: 9.0, 0.5


# 运行nebula完整数据
docker exec -it mrslam bash /home/cyw_local/MR_SLAM/run_jackal_mrslam_nebula.sh run