import cv2
import os

def resize_and_pad(image, target_size=(640, 640)):
    """
    调整图像大小并填充，使其达到目标尺寸
    """
    h, w = image.shape[:2]
    aspect_ratio = w / h

    # 计算新的宽高
    if aspect_ratio > 1:  # 宽大于高
        new_w = target_size[0]
        new_h = int(target_size[0] / aspect_ratio)
    else:  # 高大于宽
        new_w = int(target_size[1] * aspect_ratio)
        new_h = target_size[1]

    # 缩放图像
    resized_image = cv2.resize(image, (new_w, new_h))

    # 创建一个目标大小的空白图像
    padded_image = cv2.copyMakeBorder(
        resized_image,
        top=(target_size[1] - new_h) // 2,
        bottom=(target_size[1] - new_h) - (target_size[1] - new_h) // 2,
        left=(target_size[0] - new_w) // 2,
        right=(target_size[0] - new_w) - (target_size[0] - new_w) // 2,
        borderType=cv2.BORDER_CONSTANT,
        value=(0, 0, 0)  # 使用黑色填充
    )
    return padded_image

def save_image_from_camera(save_dir="saved_images"):
    # 创建保存图像的目录
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
        print(f"Created directory to save images: {save_dir}")

    # 打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("按 's' 键保存图像，按 'q' 键退出程序。")
    image_count = 0

    while True:
        # 从摄像头读取一帧图像
        ret, frame = cap.read()
        if not ret:
            print("无法读取图像")
            break

        # 显示图像
        cv2.imshow('Camera', frame)

        # 检测按键
        key = cv2.waitKey(1) & 0xFF

        # 如果按下 's' 键，保存图像
        if key == ord('s'):
            # 调整图像大小并填充
            resized_frame = resize_and_pad(frame, target_size=(640, 640))
            image_path = os.path.join(save_dir, f"image_{image_count}.jpg")
            cv2.imwrite(image_path, resized_frame)
            print(f"图像已保存到 {image_path}")
            image_count += 1

        # 如果按下 'q' 键，退出程序
        elif key == ord('q'):
            break

    # 释放摄像头资源并关闭窗口
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    save_image_from_camera()