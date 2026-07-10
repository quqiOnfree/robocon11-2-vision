# 合并启动配置，通过 0x0000 一次性发送给下位机
int16 area_type    # 0=blue, 1=red
int16 begin_type   # 0=一区, 1=二区, 2=三区坡前, 3=三区重试
int16 origin_x     # 起点 X (mm)
int16 origin_y     # 起点 Y (mm)
int16 kfs_amount               # 车内初始方块数量 0~3
int16 arena_load_kfs_amount    # 三区加载方块数量 0~2
int16 arena_delay_seconds      # 三区启动等待秒数 10~60
