import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory #获取 pkg share 绝对路径

def main():
    # 获取图片路径
    default_pic_path = get_package_share_directory("pic_python_service")+'/resource'+'/default.jpg'
    print(f"图片路径：{default_pic_path}")

    #使用cv2加载图片
    image = cv2.imread(default_pic_path)
    face_recognition.face_locations(image,number_of_times_to_upsample=1,model='hog')

    # 绘制人链
    for top,right,bottom,left in face_recognition:
        cv2.rectangle(image,(left,top),(right,bottom),(255,0,0),4)
    
    # 显示
    cv2.imshow('pic_out',image)
    cv2.waitKey(0)
    