import cv2
import numpy as np
import torch

from std_msgs.msg import String
from .emotion_detector import EmotionDetector
from .GestureDetector_rs import GestureDetector
from imutils.video import FPS
from .utils.utils import setup_logger

import os
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0' 

class EmotionGestureCompiler:
    # Versão CORRIGIDA da função __init__
    def __init__(
        self,
        com_ros: None,
        model_name: str = "resnet18.onnx",
        model_option: str = "onnx",
        backend_option: int = 0 if torch.cuda.is_available() else 1,
        providers: int = 2,  #1
        fp16: bool = False,
        num_faces: int = 1,
        train_path: str = 'Base_de_dados',
        k : int = 7,
    ):
        self.gestures_list = ['A', 'B', 'C', 'D', 'E']
        self.emotions_list = {0: "BAD", 1: "GOOD", 2: "NEUTRAL"}
        self.com_ros = com_ros

        self.logger = setup_logger(__name__)    # debug logger

        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        absolute_train_path = os.path.join(self.script_dir, train_path)

        # gesture and emotion inicialization
        self.Emotion = EmotionDetector(model_name, model_option, backend_option, providers, fp16, num_faces)
        # Passa o caminho absoluto para o GestureDetector
        self.Gesture = GestureDetector(self.gestures_list, absolute_train_path, k)

        return
    def get_emotion (self, img):
        
        height, width = img.shape[:2]

        blob = cv2.dnn.blobFromImage(
            cv2.resize(img, (300, 300)),
            1.0,
            (300, 300),
            (104.0, 177.0, 123.0),
            swapRB=False,
            crop=False,
        )

        self.Emotion.face_model.setInput(blob)
        predictions = self.Emotion.face_model.forward()
        print('Face detected')

        for i in range(predictions.shape[2]):
            if predictions[0, 0, i, 2] > 0.5:
                bbox = predictions[0, 0, i, 3:7] * np.array(
                    [width, height, width, height]
                )
                (x_min, y_min, x_max, y_max) = bbox.astype("int")
                
                print(f"Bounding box coordinates: {x_min}, {y_min}, {x_max}, {y_max}")

                # draws red rectangle
                cv2.rectangle(
                    img, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2
                )

                face = img[y_min:y_max, x_min:x_max]

                # makes prediction 
                emotion = self.Emotion.recognize_emotion(face)
                
                print(f"Emotion detected: {self.emotions_list[emotion]}\n")

                # writes emotion name   
                cv2.putText(
                    img,
                    self.emotions_list[emotion],
                    (x_min + 5, y_min - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
        try:
            return img, emotion
        except:
            return img, 0

    def video(self, video_path='realsense'):
        
        # Adicionando o backend FFMPEG para maior robustez com streams de rede
        if isinstance(video_path, str) and '://' in video_path:
            cap = cv2.VideoCapture(video_path, cv2.CAP_FFMPEG)
        else:
            cap = cv2.VideoCapture(video_path)

        self.logger.info(f"video Path: {video_path}")

        if not cap.isOpened():
            self.logger.error("Error opening video stream or file")
            return

        success, img = cap.read()
        fps = FPS().start()

        # Inicializa o contador fora do loop principal
        counter = {"GOOD": 10, "BAD": 10}

        while success:
            try:
                if not self.Gesture.resp in self.gestures_list:
                    img = self.Gesture.process_frame(img)
                    # Reseta o contador sempre que um novo gesto começa a ser formado
                    counter = {"GOOD": 10, "BAD": 10}

                else:   # Entra quando um gesto válido é capturado
                    img, emotion = self.get_emotion(img)
                    img = self.Gesture.print_data(img)
                    
                    if self.emotions_list[emotion] == "GOOD":
                        counter["GOOD"] -= 1
                        if counter["GOOD"] < 1:
                            
                            gesture_result = self.Gesture.gesture_ros()
                            msg_to_publish = String()
                            msg_to_publish.data = str(gesture_result)
                            self.com_ros.publish(msg_to_publish)
                            self.logger.info(f"Gesto '{gesture_result}' publicado no ROS.")

                            #save_file_path = os.path.join(self.script_dir, 'lists', 'gesture_txt.txt')
                            #self.Gesture.save_gesture(path=save_file_path)

                            self.Gesture.reset_pred()
                            
                        counter["BAD"] = 10

                    elif self.emotions_list[emotion] == "BAD":
                        counter["BAD"] -= 1
                        if counter["BAD"] < 1:
                            self.logger.info("DataSet Rejected")
                            self.Gesture.reset_pred()
                        
                        # Reseta o contador oposto
                        counter["GOOD"] = 10

                cv2.imshow("Capturing", img)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                fps.update()
                success, img = cap.read()

            except KeyboardInterrupt:
                break
        
        fps.stop()
        self.logger.info("Elapsed time: %.2f", fps.elapsed())
        self.logger.info("Approx. FPS: %.2f", fps.fps())

        cap.release()
        cv2.destroyAllWindows()
        return