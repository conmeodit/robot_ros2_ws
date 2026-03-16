from pathlib import Path
from typing import Dict, List


class YoloWrapper:
	def __init__(
		self,
		model_path: str,
		conf_threshold: float = 0.25,
		iou_threshold: float = 0.45,
		device: str = 'cpu',
	) -> None:
		try:
			from ultralytics import YOLO
		except Exception as exc:
			raise RuntimeError(
				'ultralytics is not installed. Please install it in your ROS2 Python environment.'
			) from exc

		path = Path(model_path)
		if not path.exists():
			raise FileNotFoundError(f'Model not found: {model_path}')

		self._model = YOLO(str(path))
		self._conf_threshold = conf_threshold
		self._iou_threshold = iou_threshold
		self._device = device

	def infer(self, image_bgr) -> List[Dict]:
		results = self._model.predict(
			source=image_bgr,
			conf=self._conf_threshold,
			iou=self._iou_threshold,
			device=self._device,
			verbose=False,
		)

		if not results:
			return []

		result = results[0]
		boxes = []

		names = result.names if hasattr(result, 'names') else {}

		for box in result.boxes:
			xyxy = box.xyxy[0].tolist()
			cls_id = int(box.cls[0].item())
			score = float(box.conf[0].item())
			label = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)

			boxes.append(
				{
					'x1': int(xyxy[0]),
					'y1': int(xyxy[1]),
					'x2': int(xyxy[2]),
					'y2': int(xyxy[3]),
					'class_id': cls_id,
					'label': label,
					'score': score,
				}
			)

		return boxes

