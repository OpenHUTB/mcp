import sys
import subprocess

def check_and_install_opencv():
    try:
        import cv2
        print(f"OpenCV version: {cv2.__version__}")
        print("OpenCV is already installed.")
        return 0
    except ImportError:
        print("OpenCV (cv2) not found, installing...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", "opencv-python"])
            print("OpenCV installed successfully.")
            return 0
        except subprocess.CalledProcessError:
            print("Error: Failed to install OpenCV")
            return 1

if __name__ == "__main__":
    sys.exit(check_and_install_opencv())
