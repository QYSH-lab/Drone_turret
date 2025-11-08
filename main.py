from ultralytics import YOLO
model = YOLO("yolov8n.pt")
model.train(
    data=r"C:\Users\105-лаб-1\PycharmProjects\nonmilitary\data\data.yaml",
    epochs=15,
    imgsz=640
)

