import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import os  # 用于路径拼接（更可靠）

def main():
    # 获取图片路径（使用os.path.join避免路径拼接错误）
    pkg_share = get_package_share_directory("pic_python_service")
    default_pic_path = os.path.join(pkg_share, "resource", "default.jpg")  # 正确拼接路径
    print(f"图片路径：{default_pic_path}")

    # 使用cv2加载图片
    image = cv2.imread(default_pic_path)
    if image is None:  # 增加图片加载失败的判断
        print(f"错误：无法加载图片，请检查路径是否正确：{default_pic_path}")
        return

    # 检测人脸位置并保存结果到变量
    face_locations = face_recognition.face_locations(
        image,
        number_of_times_to_upsample=1,
        model='hog'
    )

    # 遍历检测到的人脸位置（使用保存结果的变量face_locations，而非模块名）
    for top, right, bottom, left in face_locations:
        cv2.rectangle(image, (left, top), (right, bottom), (255, 0, 0), 4)
    
    # 显示图像
    cv2.imshow('pic_out', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()  # 关闭窗口，释放资源

if __name__ == "__main__":
    main()
