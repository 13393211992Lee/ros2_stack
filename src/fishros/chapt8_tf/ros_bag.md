### ros bag
$ ros2 bag record /turtle1/cmd_vel
$ ros2 bag play rosbag2_2025_10_11-14_27_03

$ ros2 bag -h
Commands:
  burst    Burst data from a bag
  convert  Given an input bag, write out a new bag with different settings
  info     Print information about a bag to the screen
  list     Print information about available plugins to the screen
  play     Play back ROS data from a bag
  record   Record ROS data to a bag
  reindex  Reconstruct metadata file for a bag

  Call `ros2 bag <command> -h` for more detailed usage.
 
 翻译：
$ ros2 bag -h 
Commands:  
  burst    从数据包（bag）中批量导出数据（注：快速连续地读取并输出bag中的数据，常用于数据批量处理或回放测试）
  convert  读取一个输入数据包（bag），按不同设置生成一个新的数据包（注：可转换bag的格式、压缩方式或筛选数据，生成符合需求的新bag文件）
  info     将数据包（bag）的信息打印到终端屏幕上（注：显示bag的基本信息，如录制时间、包含的话题列表、消息数量、文件大小等）
  list     将可用插件的信息打印到终端屏幕上（注：列出当前系统中支持ros2 bag功能的插件，如数据存储插件、压缩插件等）
  play     从数据包（bag）中回放ROS数据（注：模拟数据发布过程，将bag中录制的话题和消息重新发布到ROS 2系统，用于复现场景或测试节点）
  record   将ROS数据录制到数据包（bag）中（注：监听指定的ROS话题，将话题上的消息存储到bag文件中，用于数据采集和后续分析）
  reindex  为数据包（bag）重建元数据文件（注：当bag的元数据文件损坏或丢失时，重新生成索引信息，确保bag可正常读取和回放）


### Record
// 录制多个话题
$ ros2 bag record /turtle1/cmd_vel /turtle1/pose /turtle1/color_sensor
