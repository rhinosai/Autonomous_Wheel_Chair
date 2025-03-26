import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import cv2
from streamlit_webrtc import webrtc_streamer, VideoTransformerBase

# Matplotlib 설정
fig, ax = plt.subplots()
sensor = None  # 센서 데이터 (필요 시 업데이트)
im = ax.imshow(np.random.rand(10, 10), cmap='viridis')  # 이미지 초기화
text_objects = [ax.text(0, 0, '', color='white') for _ in range(3)]  # 텍스트 객체

# 애니메이션 업데이트 함수
def updatefig(frame, sensor, im, text_objects):
    # 예제: im 데이터 업데이트 (실제 데이터로 변경 가능)
    im.set_array(np.random.rand(10, 10))

    # 예제: 텍스트 업데이트
    for i, txt in enumerate(text_objects):
        txt.set_text(f"Frame: {frame}, Text {i}")

    fig.canvas.draw()

    # Matplotlib을 OpenCV 이미지로 변환
    img = np.array(fig.canvas.renderer.buffer_rgba())
    img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)  # OpenCV 형식 변환
    return img

# WebRTC용 비디오 변환 클래스
class VideoTransformer(VideoTransformerBase):
    def __init__(self):
        self.frame_num = 0  # 프레임 카운터

    def transform(self, frame):
        self.frame_num += 1
        img = updatefig(self.frame_num, sensor, im, text_objects)
        return img

# Streamlit UI
st.title("📡 Matplotlib Animation Streaming with WebRTC")

webrtc_streamer(key="example", video_transformer_factory=VideoTransformer)
